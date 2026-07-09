#!/usr/bin/env python3
"""Inspect raw D-LIO sensor bag timestamps offline.

This script reads a rosbag2 directory directly. It does not play the bag and does
not start D-LIO. It is intended for raw bags recorded from /go2w/imu and
/points_raw.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from pathlib import Path
from statistics import mean, median
from typing import Any

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2
except Exception as exc:  # pragma: no cover - environment dependent
    print("Error: ROS 2 Python bag/message modules are not available.", file=sys.stderr)
    print("Source ROS 2 Humble and the workspace before running this script.", file=sys.stderr)
    print(f"Import error: {type(exc).__name__}: {exc}", file=sys.stderr)
    sys.exit(1)


NSEC_PER_SEC = 1_000_000_000.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Offline timestamp diagnostics for raw D-LIO sensor bags."
    )
    parser.add_argument("bag", help="rosbag2 directory containing metadata.yaml")
    parser.add_argument("--imu-topic", default="/go2w/imu", help="IMU topic name")
    parser.add_argument("--lidar-topic", default="/points_raw", help="LiDAR PointCloud2 topic name")
    parser.add_argument(
        "--point-cloud-stride",
        type=int,
        default=10,
        help="Analyze per-point timestamps for every Nth LiDAR cloud.",
    )
    parser.add_argument(
        "--max-point-clouds",
        type=int,
        default=500,
        help="Maximum LiDAR clouds used for per-point timestamp analysis.",
    )
    parser.add_argument(
        "--point-stride",
        type=int,
        default=1,
        help="Read every Nth point timestamp inside sampled clouds.",
    )
    parser.add_argument(
        "--json",
        dest="json_path",
        default="",
        help="Optional path to write a machine-readable JSON report.",
    )
    return parser.parse_args()


def sec_from_msg_stamp(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / NSEC_PER_SEC


def sec_from_bag_stamp(stamp_ns: int) -> float:
    return float(stamp_ns) / NSEC_PER_SEC


def finite(values: list[float]) -> list[float]:
    return [v for v in values if math.isfinite(v)]


def percentile(sorted_values: list[float], pct: float) -> float | None:
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]
    rank = (len(sorted_values) - 1) * pct / 100.0
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return sorted_values[lo]
    frac = rank - lo
    return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac


def basic_stats(values: list[float]) -> dict[str, Any]:
    vals = sorted(finite(values))
    if not vals:
        return {"count": 0}
    return {
        "count": len(vals),
        "min": vals[0],
        "median": median(vals),
        "mean": mean(vals),
        "p95": percentile(vals, 95.0),
        "p99": percentile(vals, 99.0),
        "max": vals[-1],
    }


def diff_report(stamps: list[float]) -> dict[str, Any]:
    diffs = [b - a for a, b in zip(stamps, stamps[1:])]
    positive = [d for d in diffs if d > 0.0]
    return {
        "count": len(stamps),
        "duplicate_count": sum(1 for d in diffs if d == 0.0),
        "backward_count": sum(1 for d in diffs if d < 0.0),
        "non_increasing_count": sum(1 for d in diffs if d <= 0.0),
        "all_dt": basic_stats(diffs),
        "positive_dt": basic_stats(positive),
    }


def fmt_sec(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.6f}s"


def fmt_ms(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value * 1000.0:.3f}ms"


def fmt_hz(dt: float | None) -> str:
    if dt is None or not math.isfinite(dt) or dt <= 0.0:
        return "n/a"
    return f"{1.0 / dt:.2f}Hz"


def get_field_names(cloud: Any) -> list[str]:
    return [field.name for field in cloud.fields]


def point_tuple_value(point: Any) -> float | None:
    try:
        return float(point[0])
    except Exception:
        pass
    try:
        return float(point["timestamp"])
    except Exception:
        pass
    try:
        return float(point.timestamp)
    except Exception:
        return None


def classify_point_timestamps(header_stamp: float, stats: dict[str, Any]) -> str:
    if stats.get("count", 0) == 0:
        return "none"
    med = stats.get("median")
    max_v = stats.get("max")
    if med is None or max_v is None:
        return "unknown"
    if max_v > 1e14:
        return "large_absolute_or_nanoseconds"
    if abs(float(med) - header_stamp) < 5.0:
        return "absolute_seconds_near_header"
    if 0.0 <= float(max_v) < 2.0:
        return "relative_seconds"
    return "unknown_seconds_scale"


def sample_point_timestamps(cloud: Any, header_stamp: float, point_stride: int) -> dict[str, Any]:
    fields = get_field_names(cloud)
    result: dict[str, Any] = {
        "field_present": "timestamp" in fields,
        "fields": fields,
        "point_count": int(cloud.width) * int(cloud.height),
        "sample_count": 0,
    }
    if "timestamp" not in fields:
        return result

    values: list[float] = []
    stride = max(1, point_stride)
    try:
        for i, point in enumerate(point_cloud2.read_points(cloud, field_names=["timestamp"], skip_nans=True)):
            if i % stride != 0:
                continue
            value = point_tuple_value(point)
            if value is not None and math.isfinite(value):
                values.append(value)
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        return result

    stats = basic_stats(values)
    result.update(stats)
    result["sample_count"] = stats.get("count", 0)
    if stats.get("count", 0) > 0:
        result["span"] = stats["max"] - stats["min"]
        result["header_minus_min"] = header_stamp - stats["min"]
        result["header_minus_median"] = header_stamp - stats["median"]
        result["header_minus_max"] = header_stamp - stats["max"]
        result["classification"] = classify_point_timestamps(header_stamp, stats)
    return result


class TopicSeries:
    def __init__(self, name: str, type_name: str) -> None:
        self.name = name
        self.type_name = type_name
        self.count = 0
        self.header_stamps: list[float] = []
        self.bag_stamps: list[float] = []
        self.record_delays: list[float] = []

    def add(self, msg: Any, bag_stamp_ns: int) -> None:
        self.count += 1
        bag_stamp = sec_from_bag_stamp(bag_stamp_ns)
        self.bag_stamps.append(bag_stamp)
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            header_stamp = sec_from_msg_stamp(msg.header.stamp)
            self.header_stamps.append(header_stamp)
            self.record_delays.append(bag_stamp - header_stamp)

    def report(self) -> dict[str, Any]:
        header_range = None
        if self.header_stamps:
            header_range = {
                "start": self.header_stamps[0],
                "end": self.header_stamps[-1],
                "duration": self.header_stamps[-1] - self.header_stamps[0],
            }
        bag_range = None
        if self.bag_stamps:
            bag_range = {
                "start": self.bag_stamps[0],
                "end": self.bag_stamps[-1],
                "duration": self.bag_stamps[-1] - self.bag_stamps[0],
            }
        return {
            "name": self.name,
            "type": self.type_name,
            "count": self.count,
            "header_range": header_range,
            "bag_range": bag_range,
            "stamp_order": diff_report(self.header_stamps),
            "record_delay": basic_stats(self.record_delays),
        }


def nearest_imu_report(imu_stamps: list[float], lidar_stamps: list[float]) -> dict[str, Any]:
    if not imu_stamps or not lidar_stamps:
        return {"count": 0}

    sorted_imu = sorted(imu_stamps)
    before_gaps: list[float] = []
    after_gaps: list[float] = []
    nearest_abs: list[float] = []
    missing_before = 0
    missing_after = 0
    outside_range = 0

    for stamp in lidar_stamps:
        idx = bisect.bisect_left(sorted_imu, stamp)
        before = sorted_imu[idx - 1] if idx > 0 else None
        after = sorted_imu[idx] if idx < len(sorted_imu) else None

        if before is None:
            missing_before += 1
        else:
            before_gaps.append(stamp - before)

        if after is None:
            missing_after += 1
        else:
            after_gaps.append(after - stamp)

        if before is None or after is None:
            outside_range += 1
        candidates = []
        if before is not None:
            candidates.append(abs(stamp - before))
        if after is not None:
            candidates.append(abs(after - stamp))
        if candidates:
            nearest_abs.append(min(candidates))

    return {
        "count": len(lidar_stamps),
        "missing_before_count": missing_before,
        "missing_after_count": missing_after,
        "outside_imu_range_count": outside_range,
        "before_gap": basic_stats(before_gaps),
        "after_gap": basic_stats(after_gaps),
        "nearest_abs_gap": basic_stats(nearest_abs),
    }


def overlap_report(a: list[float], b: list[float]) -> dict[str, Any]:
    if not a or not b:
        return {"has_overlap": False}
    start = max(min(a), min(b))
    end = min(max(a), max(b))
    return {"has_overlap": end >= start, "start": start, "end": end, "duration": end - start}


def point_summary(samples: list[dict[str, Any]], imu_stamps: list[float]) -> dict[str, Any]:
    if not samples:
        return {"sampled_clouds": 0}

    usable = [s for s in samples if s.get("sample_count", 0) > 0]
    classifications: dict[str, int] = {}
    spans: list[float] = []
    header_minus_median: list[float] = []
    covered_by_imu = 0
    imu_min = min(imu_stamps) if imu_stamps else None
    imu_max = max(imu_stamps) if imu_stamps else None

    for sample in usable:
        cls = sample.get("classification", "unknown")
        classifications[cls] = classifications.get(cls, 0) + 1
        if "span" in sample:
            spans.append(sample["span"])
        if "header_minus_median" in sample:
            header_minus_median.append(sample["header_minus_median"])
        if imu_min is not None and imu_max is not None and "min" in sample and "max" in sample:
            if imu_min <= sample["min"] and sample["max"] <= imu_max:
                covered_by_imu += 1

    first_fields = samples[0].get("fields", [])
    return {
        "sampled_clouds": len(samples),
        "usable_clouds": len(usable),
        "timestamp_field_present": any(s.get("field_present", False) for s in samples),
        "fields_first_sample": first_fields,
        "classifications": classifications,
        "span": basic_stats(spans),
        "header_minus_point_median": basic_stats(header_minus_median),
        "point_time_range_covered_by_imu_count": covered_by_imu,
        "examples": usable[:5],
    }


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def analyze_bag(args: argparse.Namespace) -> dict[str, Any]:
    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.is_dir():
        raise RuntimeError(f"bag directory not found: {bag_path}")
    if not (bag_path / "metadata.yaml").is_file():
        raise RuntimeError(f"metadata.yaml not found in bag directory: {bag_path}")

    reader = open_reader(bag_path)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    missing = [topic for topic in (args.imu_topic, args.lidar_topic) if topic not in topic_types]
    if missing:
        found = "\n".join(f"  {name}: {type_name}" for name, type_name in sorted(topic_types.items()))
        raise RuntimeError(f"missing required topic(s): {', '.join(missing)}\nTopics found:\n{found}")

    imu_msg_type = get_message(topic_types[args.imu_topic])
    lidar_msg_type = get_message(topic_types[args.lidar_topic])
    imu = TopicSeries(args.imu_topic, topic_types[args.imu_topic])
    lidar = TopicSeries(args.lidar_topic, topic_types[args.lidar_topic])

    point_samples: list[dict[str, Any]] = []
    lidar_seen = 0
    point_stride = max(1, args.point_stride)
    cloud_stride = max(1, args.point_cloud_stride)
    max_clouds = max(0, args.max_point_clouds)

    while reader.has_next():
        topic, serialized, bag_stamp_ns = reader.read_next()
        if topic == args.imu_topic:
            msg = deserialize_message(serialized, imu_msg_type)
            imu.add(msg, bag_stamp_ns)
        elif topic == args.lidar_topic:
            msg = deserialize_message(serialized, lidar_msg_type)
            lidar.add(msg, bag_stamp_ns)
            lidar_seen += 1
            if (lidar_seen - 1) % cloud_stride == 0 and len(point_samples) < max_clouds:
                header_stamp = sec_from_msg_stamp(msg.header.stamp)
                sample = sample_point_timestamps(msg, header_stamp, point_stride)
                sample["cloud_index"] = lidar_seen
                sample["header_stamp"] = header_stamp
                point_samples.append(sample)

    report = {
        "bag": str(bag_path),
        "topics": {
            args.imu_topic: imu.report(),
            args.lidar_topic: lidar.report(),
        },
        "overlap": overlap_report(imu.header_stamps, lidar.header_stamps),
        "lidar_to_imu": nearest_imu_report(imu.header_stamps, lidar.header_stamps),
        "point_timestamps": point_summary(point_samples, imu.header_stamps),
    }
    return report


def print_topic_report(topic: dict[str, Any]) -> None:
    name = topic["name"]
    count = topic["count"]
    print(f"\n[{name}]")
    print(f"  type: {topic['type']}")
    print(f"  messages: {count}")

    header_range = topic.get("header_range")
    if header_range:
        print(
            "  header stamp range: "
            f"{fmt_sec(header_range['start'])} -> {fmt_sec(header_range['end'])} "
            f"(duration {fmt_sec(header_range['duration'])})"
        )
    bag_range = topic.get("bag_range")
    if bag_range:
        print(
            "  bag record range:  "
            f"{fmt_sec(bag_range['start'])} -> {fmt_sec(bag_range['end'])} "
            f"(duration {fmt_sec(bag_range['duration'])})"
        )

    order = topic["stamp_order"]
    positive_dt = order["positive_dt"]
    med_dt = positive_dt.get("median")
    print(
        "  header dt: "
        f"median {fmt_ms(med_dt)}, p95 {fmt_ms(positive_dt.get('p95'))}, "
        f"estimated rate {fmt_hz(med_dt)}"
    )
    print(
        "  stamp order: "
        f"duplicates={order['duplicate_count']}, backward={order['backward_count']}, "
        f"non_increasing={order['non_increasing_count']}"
    )

    delay = topic["record_delay"]
    print(
        "  bag_time - header_stamp: "
        f"median {fmt_ms(delay.get('median'))}, p95 {fmt_ms(delay.get('p95'))}, "
        f"max {fmt_ms(delay.get('max'))}"
    )


def print_report(report: dict[str, Any], imu_topic: str, lidar_topic: str) -> None:
    print(f"Bag: {report['bag']}")
    print_topic_report(report["topics"][imu_topic])
    print_topic_report(report["topics"][lidar_topic])

    overlap = report["overlap"]
    print("\n[IMU/LiDAR header stamp overlap]")
    if overlap.get("has_overlap"):
        print(
            f"  overlap: {fmt_sec(overlap['start'])} -> {fmt_sec(overlap['end'])} "
            f"(duration {fmt_sec(overlap['duration'])})"
        )
    else:
        print("  no overlap")

    sync = report["lidar_to_imu"]
    print("\n[LiDAR frames vs nearest IMU samples]")
    if sync.get("count", 0) == 0:
        print("  no data")
    else:
        nearest = sync["nearest_abs_gap"]
        before = sync["before_gap"]
        after = sync["after_gap"]
        print(f"  lidar frames checked: {sync['count']}")
        print(
            "  nearest |lidar - imu|: "
            f"median {fmt_ms(nearest.get('median'))}, p95 {fmt_ms(nearest.get('p95'))}, "
            f"max {fmt_ms(nearest.get('max'))}"
        )
        print(
            "  previous IMU gap: "
            f"median {fmt_ms(before.get('median'))}, p95 {fmt_ms(before.get('p95'))}, "
            f"max {fmt_ms(before.get('max'))}"
        )
        print(
            "  next IMU gap:     "
            f"median {fmt_ms(after.get('median'))}, p95 {fmt_ms(after.get('p95'))}, "
            f"max {fmt_ms(after.get('max'))}"
        )
        print(
            "  outside IMU range: "
            f"{sync['outside_imu_range_count']} "
            f"(missing_before={sync['missing_before_count']}, missing_after={sync['missing_after_count']})"
        )

    point = report["point_timestamps"]
    print("\n[Hesai PointCloud2 per-point timestamp field]")
    print(f"  sampled clouds: {point.get('sampled_clouds', 0)}")
    print(f"  usable clouds:  {point.get('usable_clouds', 0)}")
    print(f"  timestamp field present: {point.get('timestamp_field_present', False)}")
    if point.get("fields_first_sample"):
        print(f"  first sample fields: {', '.join(point['fields_first_sample'])}")
    if point.get("usable_clouds", 0) > 0:
        span = point["span"]
        hm = point["header_minus_point_median"]
        print(f"  classification counts: {point.get('classifications', {})}")
        print(
            "  point timestamp span: "
            f"median {fmt_ms(span.get('median'))}, p95 {fmt_ms(span.get('p95'))}, "
            f"max {fmt_ms(span.get('max'))}"
        )
        print(
            "  header_stamp - point_median: "
            f"median {fmt_ms(hm.get('median'))}, p95 {fmt_ms(hm.get('p95'))}, "
            f"max {fmt_ms(hm.get('max'))}"
        )
        print(
            "  point ranges covered by IMU range: "
            f"{point.get('point_time_range_covered_by_imu_count', 0)} / {point.get('usable_clouds', 0)}"
        )


def main() -> int:
    args = parse_args()
    try:
        report = analyze_bag(args)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print_report(report, args.imu_topic, args.lidar_topic)

    if args.json_path:
        out = Path(args.json_path).expanduser().resolve()
        with out.open("w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, sort_keys=True)
            f.write("\n")
        print(f"\nJSON report written: {out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
