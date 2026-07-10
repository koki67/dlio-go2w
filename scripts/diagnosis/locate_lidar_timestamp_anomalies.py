#!/usr/bin/env python3
"""Locate exact LiDAR timestamp anomalies in a rosbag2 raw sensor bag.

This script reads an existing rosbag2 directory directly. It is intended as a
focused follow-up to check_raw_bag_timestamps.py when that summary reports
LiDAR header stamp backward/non-increasing frames or large bag record delays.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from statistics import median
from typing import Any

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except Exception as exc:  # pragma: no cover - environment dependent
    print("Error: ROS 2 Python bag/message modules are not available.", file=sys.stderr)
    print("Source ROS 2 Humble and the workspace before running this script.", file=sys.stderr)
    print(f"Import error: {type(exc).__name__}: {exc}", file=sys.stderr)
    sys.exit(1)


NSEC_PER_SEC = 1_000_000_000


@dataclass
class Frame:
    index: int
    header_ns: int
    bag_ns: int
    delay_ns: int
    prev_dt_ns: int | None = None
    next_dt_ns: int | None = None
    nearest_imu_abs_ns: int | None = None
    nearest_imu_signed_ns: int | None = None
    imu_prev_gap_ns: int | None = None
    imu_next_gap_ns: int | None = None
    flags: list[str] = field(default_factory=list)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Locate exact LiDAR timestamp anomalies in an existing rosbag2 bag."
    )
    parser.add_argument("bag", help="rosbag2 directory containing metadata.yaml")
    parser.add_argument("--lidar-topic", default="/points_raw", help="LiDAR PointCloud2 topic")
    parser.add_argument("--imu-topic", default="/go2w/imu", help="Optional IMU topic for nearest-sample checks")
    parser.add_argument(
        "--record-delay-threshold-ms",
        type=float,
        default=500.0,
        help="Flag frames where bag_time - header.stamp exceeds this value.",
    )
    parser.add_argument(
        "--delay-jump-threshold-ms",
        type=float,
        default=500.0,
        help="Flag frames where record delay changes by more than this value from the previous LiDAR frame.",
    )
    parser.add_argument(
        "--imu-gap-threshold-ms",
        type=float,
        default=10.0,
        help="Flag LiDAR frames whose nearest IMU sample is farther than this value.",
    )
    parser.add_argument(
        "--context",
        type=int,
        default=2,
        help="Number of neighboring LiDAR frames to print around each anomaly.",
    )
    parser.add_argument(
        "--max-groups",
        type=int,
        default=20,
        help="Maximum anomaly context groups printed to the terminal.",
    )
    parser.add_argument(
        "--top-delays",
        type=int,
        default=10,
        help="Number of largest record-delay frames to print.",
    )
    parser.add_argument(
        "--json",
        dest="json_path",
        default="",
        help="Optional path to write a machine-readable JSON report.",
    )
    return parser.parse_args()


def ns_from_msg_stamp(stamp: Any) -> int:
    return int(stamp.sec) * NSEC_PER_SEC + int(stamp.nanosec)


def fmt_sec(ns: int | None) -> str:
    if ns is None:
        return "n/a"
    return f"{ns / NSEC_PER_SEC:.9f}s"


def fmt_ms(ns: int | None) -> str:
    if ns is None:
        return "n/a"
    return f"{ns / 1_000_000.0:.3f}ms"


def percentile(sorted_values: list[int], pct: float) -> int | None:
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
    return int(round(sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac))


def stats(values: list[int]) -> dict[str, Any]:
    vals = sorted(values)
    if not vals:
        return {"count": 0}
    return {
        "count": len(vals),
        "min_ns": vals[0],
        "median_ns": int(median(vals)),
        "p95_ns": percentile(vals, 95.0),
        "p99_ns": percentile(vals, 99.0),
        "max_ns": vals[-1],
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


def find_nearest_imu(frame: Frame, imu_stamps: list[int]) -> None:
    if not imu_stamps:
        return

    idx = bisect.bisect_left(imu_stamps, frame.header_ns)
    before = imu_stamps[idx - 1] if idx > 0 else None
    after = imu_stamps[idx] if idx < len(imu_stamps) else None

    if before is not None:
        frame.imu_prev_gap_ns = frame.header_ns - before
    if after is not None:
        frame.imu_next_gap_ns = after - frame.header_ns

    candidates: list[tuple[int, int]] = []
    if before is not None:
        candidates.append((abs(frame.header_ns - before), before - frame.header_ns))
    if after is not None:
        candidates.append((abs(after - frame.header_ns), after - frame.header_ns))
    if candidates:
        nearest_abs, nearest_signed = min(candidates, key=lambda item: item[0])
        frame.nearest_imu_abs_ns = nearest_abs
        frame.nearest_imu_signed_ns = nearest_signed


def read_bag(args: argparse.Namespace) -> tuple[Path, list[Frame], list[int], dict[str, str]]:
    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.is_dir():
        raise RuntimeError(f"bag directory not found: {bag_path}")
    if not (bag_path / "metadata.yaml").is_file():
        raise RuntimeError(f"metadata.yaml not found in bag directory: {bag_path}")

    reader = open_reader(bag_path)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if args.lidar_topic not in topic_types:
        found = "\n".join(f"  {name}: {type_name}" for name, type_name in sorted(topic_types.items()))
        raise RuntimeError(f"missing LiDAR topic: {args.lidar_topic}\nTopics found:\n{found}")

    lidar_msg_type = get_message(topic_types[args.lidar_topic])
    imu_msg_type = get_message(topic_types[args.imu_topic]) if args.imu_topic in topic_types else None

    frames: list[Frame] = []
    imu_stamps: list[int] = []

    while reader.has_next():
        topic, serialized, bag_stamp_ns = reader.read_next()
        if topic == args.lidar_topic:
            msg = deserialize_message(serialized, lidar_msg_type)
            if not hasattr(msg, "header") or not hasattr(msg.header, "stamp"):
                raise RuntimeError(f"LiDAR topic has no header.stamp: {args.lidar_topic}")
            header_ns = ns_from_msg_stamp(msg.header.stamp)
            frames.append(
                Frame(
                    index=len(frames) + 1,
                    header_ns=header_ns,
                    bag_ns=int(bag_stamp_ns),
                    delay_ns=int(bag_stamp_ns) - header_ns,
                )
            )
        elif imu_msg_type is not None and topic == args.imu_topic:
            msg = deserialize_message(serialized, imu_msg_type)
            if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
                imu_stamps.append(ns_from_msg_stamp(msg.header.stamp))

    imu_stamps.sort()
    return bag_path, frames, imu_stamps, topic_types


def mark_anomalies(
    frames: list[Frame],
    imu_stamps: list[int],
    record_delay_threshold_ns: int,
    delay_jump_threshold_ns: int,
    imu_gap_threshold_ns: int,
) -> None:
    for i, frame in enumerate(frames):
        if i > 0:
            prev = frames[i - 1]
            frame.prev_dt_ns = frame.header_ns - prev.header_ns
            if frame.prev_dt_ns < 0:
                frame.flags.append("backward")
            elif frame.prev_dt_ns == 0:
                frame.flags.append("duplicate")
            if frame.prev_dt_ns <= 0:
                frame.flags.append("non_increasing")

            delay_jump = frame.delay_ns - prev.delay_ns
            if abs(delay_jump) > delay_jump_threshold_ns:
                frame.flags.append("delay_jump")

        if i + 1 < len(frames):
            frame.next_dt_ns = frames[i + 1].header_ns - frame.header_ns

        if frame.delay_ns > record_delay_threshold_ns:
            frame.flags.append("record_delay_outlier")

        find_nearest_imu(frame, imu_stamps)
        if frame.nearest_imu_abs_ns is None:
            frame.flags.append("outside_imu_range")
        elif frame.nearest_imu_abs_ns > imu_gap_threshold_ns:
            frame.flags.append("imu_gap_outlier")


def group_indices(indices: list[int], context: int, total: int) -> list[tuple[int, int]]:
    ranges: list[tuple[int, int]] = []
    for idx in sorted(set(indices)):
        start = max(1, idx - context)
        end = min(total, idx + context)
        if not ranges or start > ranges[-1][1] + 1:
            ranges.append((start, end))
        else:
            ranges[-1] = (ranges[-1][0], max(ranges[-1][1], end))
    return ranges


def frame_to_dict(frame: Frame) -> dict[str, Any]:
    return {
        "index": frame.index,
        "header_stamp_ns": frame.header_ns,
        "header_stamp_sec": frame.header_ns / NSEC_PER_SEC,
        "bag_time_ns": frame.bag_ns,
        "bag_time_sec": frame.bag_ns / NSEC_PER_SEC,
        "record_delay_ns": frame.delay_ns,
        "record_delay_ms": frame.delay_ns / 1_000_000.0,
        "prev_dt_ns": frame.prev_dt_ns,
        "prev_dt_ms": None if frame.prev_dt_ns is None else frame.prev_dt_ns / 1_000_000.0,
        "next_dt_ns": frame.next_dt_ns,
        "next_dt_ms": None if frame.next_dt_ns is None else frame.next_dt_ns / 1_000_000.0,
        "nearest_imu_abs_ns": frame.nearest_imu_abs_ns,
        "nearest_imu_abs_ms": None
        if frame.nearest_imu_abs_ns is None
        else frame.nearest_imu_abs_ns / 1_000_000.0,
        "nearest_imu_signed_ns": frame.nearest_imu_signed_ns,
        "nearest_imu_signed_ms": None
        if frame.nearest_imu_signed_ns is None
        else frame.nearest_imu_signed_ns / 1_000_000.0,
        "imu_prev_gap_ns": frame.imu_prev_gap_ns,
        "imu_next_gap_ns": frame.imu_next_gap_ns,
        "flags": frame.flags,
    }


def build_report(
    bag_path: Path,
    args: argparse.Namespace,
    frames: list[Frame],
    imu_stamps: list[int],
    topic_types: dict[str, str],
) -> dict[str, Any]:
    dts = [frame.prev_dt_ns for frame in frames if frame.prev_dt_ns is not None]
    positive_dts = [dt for dt in dts if dt > 0]
    delays = [frame.delay_ns for frame in frames]
    nearest = [frame.nearest_imu_abs_ns for frame in frames if frame.nearest_imu_abs_ns is not None]

    anomaly_frames = [frame for frame in frames if frame.flags]
    backward_frames = [frame for frame in frames if "backward" in frame.flags]
    duplicate_frames = [frame for frame in frames if "duplicate" in frame.flags]
    non_increasing_frames = [frame for frame in frames if "non_increasing" in frame.flags]
    record_delay_outliers = [frame for frame in frames if "record_delay_outlier" in frame.flags]
    delay_jumps = [frame for frame in frames if "delay_jump" in frame.flags]
    imu_gap_outliers = [frame for frame in frames if "imu_gap_outlier" in frame.flags]

    top_delays = sorted(frames, key=lambda frame: frame.delay_ns, reverse=True)[: max(0, args.top_delays)]

    return {
        "bag": str(bag_path),
        "lidar_topic": args.lidar_topic,
        "lidar_type": topic_types.get(args.lidar_topic, ""),
        "imu_topic": args.imu_topic,
        "imu_type": topic_types.get(args.imu_topic, ""),
        "lidar_frame_count": len(frames),
        "imu_sample_count": len(imu_stamps),
        "thresholds": {
            "record_delay_threshold_ms": args.record_delay_threshold_ms,
            "delay_jump_threshold_ms": args.delay_jump_threshold_ms,
            "imu_gap_threshold_ms": args.imu_gap_threshold_ms,
        },
        "header_dt": {
            "all": stats(dts),
            "positive": stats(positive_dts),
            "backward_count": len(backward_frames),
            "duplicate_count": len(duplicate_frames),
            "non_increasing_count": len(non_increasing_frames),
        },
        "record_delay": stats(delays),
        "nearest_imu_abs_gap": stats(nearest),
        "counts": {
            "anomaly_frames": len(anomaly_frames),
            "backward": len(backward_frames),
            "duplicate": len(duplicate_frames),
            "non_increasing": len(non_increasing_frames),
            "record_delay_outlier": len(record_delay_outliers),
            "delay_jump": len(delay_jumps),
            "imu_gap_outlier": len(imu_gap_outliers),
        },
        "anomaly_frames": [frame_to_dict(frame) for frame in anomaly_frames],
        "top_record_delay_frames": [frame_to_dict(frame) for frame in top_delays],
    }


def print_frame_row(frame: Frame) -> None:
    flags = ",".join(frame.flags) if frame.flags else "-"
    print(
        f"  {frame.index:6d}  "
        f"{fmt_sec(frame.header_ns):>20}  "
        f"{fmt_ms(frame.prev_dt_ns):>12}  "
        f"{fmt_sec(frame.bag_ns):>20}  "
        f"{fmt_ms(frame.delay_ns):>12}  "
        f"{fmt_ms(frame.nearest_imu_abs_ns):>12}  "
        f"{flags}"
    )


def print_summary(report: dict[str, Any]) -> None:
    print(f"Bag: {report['bag']}")
    print(f"LiDAR: {report['lidar_topic']} ({report['lidar_type']})")
    if report["imu_type"]:
        print(f"IMU:   {report['imu_topic']} ({report['imu_type']})")
    else:
        print(f"IMU:   {report['imu_topic']} (not found; IMU gap checks disabled)")

    header = report["header_dt"]
    positive = header["positive"]
    delay = report["record_delay"]
    imu_gap = report["nearest_imu_abs_gap"]

    print("\n[Summary]")
    print(f"  LiDAR frames: {report['lidar_frame_count']}")
    print(f"  IMU samples:  {report['imu_sample_count']}")
    print(
        "  header dt: "
        f"median {fmt_ms(positive.get('median_ns'))}, "
        f"p95 {fmt_ms(positive.get('p95_ns'))}, "
        f"backward={header['backward_count']}, "
        f"duplicate={header['duplicate_count']}, "
        f"non_increasing={header['non_increasing_count']}"
    )
    print(
        "  bag_time - header_stamp: "
        f"median {fmt_ms(delay.get('median_ns'))}, "
        f"p95 {fmt_ms(delay.get('p95_ns'))}, "
        f"p99 {fmt_ms(delay.get('p99_ns'))}, "
        f"max {fmt_ms(delay.get('max_ns'))}"
    )
    if imu_gap.get("count", 0) > 0:
        print(
            "  nearest |lidar - imu|: "
            f"median {fmt_ms(imu_gap.get('median_ns'))}, "
            f"p95 {fmt_ms(imu_gap.get('p95_ns'))}, "
            f"max {fmt_ms(imu_gap.get('max_ns'))}"
        )

    counts = report["counts"]
    print("\n[Anomaly counts]")
    print(f"  anomaly frames:       {counts['anomaly_frames']}")
    print(f"  backward:             {counts['backward']}")
    print(f"  duplicate:            {counts['duplicate']}")
    print(f"  non_increasing:       {counts['non_increasing']}")
    print(f"  record_delay_outlier: {counts['record_delay_outlier']}")
    print(f"  delay_jump:           {counts['delay_jump']}")
    print(f"  imu_gap_outlier:      {counts['imu_gap_outlier']}")


def print_top_delays(report: dict[str, Any]) -> None:
    rows = report["top_record_delay_frames"]
    if not rows:
        return

    print("\n[Top record delays]")
    print("   index          header_stamp       prev_dt      bag_time          delay    imu_gap  flags")
    for row in rows:
        frame = Frame(
            index=row["index"],
            header_ns=row["header_stamp_ns"],
            bag_ns=row["bag_time_ns"],
            delay_ns=row["record_delay_ns"],
            prev_dt_ns=row["prev_dt_ns"],
            next_dt_ns=row["next_dt_ns"],
            nearest_imu_abs_ns=row["nearest_imu_abs_ns"],
            nearest_imu_signed_ns=row["nearest_imu_signed_ns"],
            imu_prev_gap_ns=row["imu_prev_gap_ns"],
            imu_next_gap_ns=row["imu_next_gap_ns"],
            flags=row["flags"],
        )
        print_frame_row(frame)


def print_context_groups(frames: list[Frame], args: argparse.Namespace) -> None:
    anomaly_indices = [frame.index for frame in frames if frame.flags]
    if not anomaly_indices:
        print("\n[Anomaly context]")
        print("  no anomaly frames found with the current thresholds")
        return

    ranges = group_indices(anomaly_indices, max(0, args.context), len(frames))
    print("\n[Anomaly context]")
    print("   index          header_stamp       prev_dt      bag_time          delay    imu_gap  flags")
    for group_no, (start, end) in enumerate(ranges[: max(0, args.max_groups)], start=1):
        if group_no > 1:
            print("")
        for idx in range(start, end + 1):
            print_frame_row(frames[idx - 1])

    remaining = len(ranges) - max(0, args.max_groups)
    if remaining > 0:
        print(f"\n  ... {remaining} more context group(s) not printed; use --max-groups to show more.")


def main() -> int:
    args = parse_args()
    try:
        bag_path, frames, imu_stamps, topic_types = read_bag(args)
        if not frames:
            raise RuntimeError(f"no messages found on LiDAR topic: {args.lidar_topic}")

        mark_anomalies(
            frames,
            imu_stamps,
            int(args.record_delay_threshold_ms * 1_000_000.0),
            int(args.delay_jump_threshold_ms * 1_000_000.0),
            int(args.imu_gap_threshold_ms * 1_000_000.0),
        )
        report = build_report(bag_path, args, frames, imu_stamps, topic_types)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print_summary(report)
    print_top_delays(report)
    print_context_groups(frames, args)

    if args.json_path:
        out = Path(args.json_path).expanduser().resolve()
        with out.open("w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, sort_keys=True)
            f.write("\n")
        print(f"\nJSON report written: {out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
