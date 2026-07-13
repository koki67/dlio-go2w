#!/usr/bin/env python3
"""Create a D-LIO-only rosbag with long recording gaps compressed safely.

This tool is for a raw bag made by concatenating independently stopped
recordings.  It keeps each recorded segment internally unchanged, but shifts
each later segment earlier in time.  The same shift is applied to rosbag
record timestamps, IMU headers, PointCloud2 headers, and the Hesai absolute
per-point ``timestamp`` field used by D-LIO deskewing.

The source bag is never modified.  The output intentionally contains only the
two raw D-LIO input topics, so it cannot accidentally give unrelated recorded
topics a misleading time base.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import shutil
import struct
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
except Exception as exc:  # pragma: no cover - environment dependent
    print("Error: ROS 2 Python bag/message modules are not available.", file=sys.stderr)
    print("Source ROS 2 Humble and the workspace before running this script.", file=sys.stderr)
    print(f"Import error: {type(exc).__name__}: {exc}", file=sys.stderr)
    sys.exit(1)


NSEC_PER_SEC = 1_000_000_000
FLOAT32 = 7
FLOAT64 = 8


@dataclass(frozen=True)
class SensorEvent:
    """One required D-LIO message observed in the input reader order."""

    reader_index: int
    topic: str
    header_ns: int
    bag_ns: int


@dataclass
class Segment:
    """A continuous interval of IMU and LiDAR headers."""

    index: int
    first_header_ns: int
    last_header_ns: int
    first_bag_ns: int
    last_bag_ns: int
    message_count: int
    gap_before_ns: int | None = None
    additional_offset_ns: int = 0
    cumulative_offset_ns: int = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compress long stopped-recording gaps in a raw D-LIO rosbag."
    )
    parser.add_argument("input_bag", help="Input rosbag2 directory containing metadata.yaml")
    parser.add_argument(
        "output_bag",
        nargs="?",
        help="Output rosbag2 directory. Default: <input_bag>_dlio_rebased",
    )
    parser.add_argument("--imu-topic", default="/go2w/imu", help="IMU topic with header.stamp")
    parser.add_argument(
        "--lidar-topic", default="/points_raw", help="PointCloud2 topic with header.stamp"
    )
    parser.add_argument(
        "--gap-threshold-sec",
        type=float,
        default=1.0,
        help="A larger gap in the combined IMU/LiDAR header timeline starts a new segment (default: 1.0).",
    )
    parser.add_argument(
        "--bridge-gap-ms",
        type=float,
        default=5.0,
        help="Minimum gap left between adjacent segments in both header and bag time (default: 5.0).",
    )
    parser.add_argument(
        "--timestamp-field",
        default="timestamp",
        help="Absolute per-point PointCloud2 time field that must be shifted with the LiDAR header (default: timestamp).",
    )
    parser.add_argument(
        "--max-point-header-offset-sec",
        type=float,
        default=5.0,
        help="Values farther than this from the cloud header are retained unchanged as invalid/sentinel point times (default: 5.0).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Inspect planned segments and offsets without creating an output bag.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Remove the output directory first if it already exists.",
    )
    parser.add_argument(
        "--json",
        dest="json_path",
        default="",
        help="Optional path to write a machine-readable report.",
    )
    return parser.parse_args()


def ns_from_msg_stamp(stamp: Any) -> int:
    return int(stamp.sec) * NSEC_PER_SEC + int(stamp.nanosec)


def set_msg_stamp(stamp: Any, stamp_ns: int) -> None:
    if stamp_ns < 0:
        raise RuntimeError(f"rebased timestamp became negative: {stamp_ns}")
    stamp.sec = stamp_ns // NSEC_PER_SEC
    stamp.nanosec = stamp_ns % NSEC_PER_SEC


def fmt_sec(ns: int | None) -> str:
    if ns is None:
        return "n/a"
    return f"{ns / NSEC_PER_SEC:.9f}s"


def fmt_ms(ns: int | None) -> str:
    if ns is None:
        return "n/a"
    return f"{ns / 1_000_000.0:.3f}ms"


def default_output_path(input_bag: Path) -> Path:
    return input_bag.with_name(f"{input_bag.name}_dlio_rebased")


def validate_args(args: argparse.Namespace) -> None:
    if args.gap_threshold_sec <= 0.0:
        raise RuntimeError("--gap-threshold-sec must be positive")
    if args.bridge_gap_ms < 0.0:
        raise RuntimeError("--bridge-gap-ms must be non-negative")
    if args.max_point_header_offset_sec <= 0.0:
        raise RuntimeError("--max-point-header-offset-sec must be positive")


def validate_paths(input_bag: Path, output_bag: Path, force: bool, dry_run: bool) -> None:
    if not input_bag.is_dir():
        raise RuntimeError(f"input bag directory not found: {input_bag}")
    if not (input_bag / "metadata.yaml").is_file():
        raise RuntimeError(f"metadata.yaml not found in input bag directory: {input_bag}")
    if output_bag.resolve() == input_bag.resolve():
        raise RuntimeError("output bag must be different from input bag")
    if dry_run:
        return
    if output_bag.exists():
        if not force:
            raise RuntimeError(f"output path already exists: {output_bag} (use --force to replace it)")
        shutil.rmtree(output_bag)


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def open_writer(output_path: Path, topics: list[Any]) -> rosbag2_py.SequentialWriter:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    for topic in topics:
        writer.create_topic(topic)
    return writer


def collect_events(
    input_bag: Path, imu_topic: str, lidar_topic: str
) -> tuple[list[SensorEvent], dict[str, Any], dict[str, str], list[Any]]:
    reader = open_reader(input_bag)
    all_topics = reader.get_all_topics_and_types()
    topic_types = {topic.name: topic.type for topic in all_topics}
    required_topics = (imu_topic, lidar_topic)
    missing = [topic for topic in required_topics if topic not in topic_types]
    if missing:
        found = "\n".join(f"  {name}: {type_name}" for name, type_name in sorted(topic_types.items()))
        raise RuntimeError(f"missing required topic(s): {', '.join(missing)}\nTopics found:\n{found}")

    msg_types = {topic: get_message(topic_types[topic]) for topic in required_topics}
    events: list[SensorEvent] = []
    counts = {topic: 0 for topic in required_topics}
    reader_index = 0
    while reader.has_next():
        topic, serialized, bag_ns_raw = reader.read_next()
        if topic not in msg_types:
            continue
        msg = deserialize_message(serialized, msg_types[topic])
        if not hasattr(msg, "header") or not hasattr(msg.header, "stamp"):
            raise RuntimeError(f"topic has no header.stamp: {topic}")
        events.append(
            SensorEvent(
                reader_index=reader_index,
                topic=topic,
                header_ns=ns_from_msg_stamp(msg.header.stamp),
                bag_ns=int(bag_ns_raw),
            )
        )
        counts[topic] += 1
        reader_index += 1

    if not events:
        raise RuntimeError("input bag contains no required D-LIO messages")

    output_topics = [topic for topic in all_topics if topic.name in msg_types]
    return events, {"input_message_counts": counts}, topic_types, output_topics


def build_segments(events: list[SensorEvent], gap_threshold_ns: int, bridge_gap_ns: int) -> list[Segment]:
    """Split the union of sensor header stamps at genuine stopped-recording gaps."""

    ordered = sorted(events, key=lambda event: (event.header_ns, event.reader_index))
    grouped: list[list[SensorEvent]] = [[]]
    for event in ordered:
        previous = grouped[-1][-1] if grouped[-1] else None
        if previous is not None and event.header_ns - previous.header_ns > gap_threshold_ns:
            grouped.append([])
        grouped[-1].append(event)

    segments: list[Segment] = []
    cumulative_offset_ns = 0
    for index, group in enumerate(grouped):
        headers = [event.header_ns for event in group]
        bags = [event.bag_ns for event in group]
        segment = Segment(
            index=index,
            first_header_ns=min(headers),
            last_header_ns=max(headers),
            first_bag_ns=min(bags),
            last_bag_ns=max(bags),
            message_count=len(group),
        )
        if segments:
            previous = segments[-1]
            segment.gap_before_ns = segment.first_header_ns - previous.last_header_ns
            header_offset_limit = segment.gap_before_ns - bridge_gap_ns
            bag_offset_limit = segment.first_bag_ns - previous.last_bag_ns - bridge_gap_ns
            additional_offset_ns = min(header_offset_limit, bag_offset_limit)
            if additional_offset_ns < 0:
                raise RuntimeError(
                    "cannot preserve monotonic header and bag time at a detected boundary: "
                    f"segment {index}, header limit={fmt_ms(header_offset_limit)}, "
                    f"bag limit={fmt_ms(bag_offset_limit)}"
                )
            segment.additional_offset_ns = additional_offset_ns
            cumulative_offset_ns += additional_offset_ns
        segment.cumulative_offset_ns = cumulative_offset_ns
        segments.append(segment)
    return segments


def offsets_for_events(events: list[SensorEvent], segments: list[Segment]) -> list[int]:
    starts = [segment.first_header_ns for segment in segments]
    offsets: list[int] = []
    for event in events:
        index = bisect.bisect_right(starts, event.header_ns) - 1
        if index < 0:
            raise RuntimeError(f"could not assign event to a segment: {event}")
        offsets.append(segments[index].cumulative_offset_ns)
    return offsets


def validate_output_order(events: list[SensorEvent], offsets: list[int]) -> None:
    """Ensure replay timestamps remain monotonic after the piecewise shift."""

    previous_output_bag_ns: int | None = None
    for event, offset_ns in zip(events, offsets):
        output_bag_ns = event.bag_ns - offset_ns
        if previous_output_bag_ns is not None and output_bag_ns < previous_output_bag_ns:
            raise RuntimeError(
                "rebasing would make rosbag playback order go backwards at "
                f"reader event {event.reader_index}: {fmt_sec(output_bag_ns)} < "
                f"{fmt_sec(previous_output_bag_ns)}"
            )
        previous_output_bag_ns = output_bag_ns


def point_timestamp_field(cloud: Any, field_name: str) -> Any:
    for field in cloud.fields:
        if field.name == field_name:
            if int(field.count) != 1:
                raise RuntimeError(
                    f"PointCloud2 field '{field_name}' has count={field.count}; scalar timestamp required"
                )
            if int(field.datatype) not in (FLOAT32, FLOAT64):
                raise RuntimeError(
                    f"PointCloud2 field '{field_name}' has datatype={field.datatype}; "
                    "FLOAT32 or FLOAT64 required"
                )
            return field
    raise RuntimeError(f"PointCloud2 timestamp field not found: {field_name}")


def rebase_point_timestamps(
    cloud: Any,
    offset_ns: int,
    field_name: str,
    max_header_offset_sec: float,
) -> tuple[int, int]:
    """Shift absolute Hesai per-point times in-place; keep invalid sentinels intact."""

    field = point_timestamp_field(cloud, field_name)
    header_sec = ns_from_msg_stamp(cloud.header.stamp) / NSEC_PER_SEC
    offset_sec = offset_ns / NSEC_PER_SEC
    byte_order = ">" if bool(cloud.is_bigendian) else "<"
    value_format = "d" if int(field.datatype) == FLOAT64 else "f"
    value_struct = struct.Struct(byte_order + value_format)
    data = bytearray(cloud.data)
    if len(data) < int(cloud.row_step) * int(cloud.height):
        raise RuntimeError("PointCloud2 data is shorter than row_step * height")

    rebased = 0
    preserved = 0
    for row in range(int(cloud.height)):
        row_start = row * int(cloud.row_step)
        for column in range(int(cloud.width)):
            position = row_start + column * int(cloud.point_step) + int(field.offset)
            value = value_struct.unpack_from(data, position)[0]
            if not math.isfinite(value) or abs(value - header_sec) > max_header_offset_sec:
                preserved += 1
                continue
            value_struct.pack_into(data, position, value - offset_sec)
            rebased += 1
    cloud.data = bytes(data)
    return rebased, preserved


def write_rebased_bag(
    input_bag: Path,
    output_bag: Path,
    output_topics: list[Any],
    topic_types: dict[str, str],
    events: list[SensorEvent],
    offsets: list[int],
    args: argparse.Namespace,
) -> dict[str, Any]:
    reader = open_reader(input_bag)
    writer = open_writer(output_bag, output_topics)
    required_topics = {args.imu_topic, args.lidar_topic}
    msg_types = {topic: get_message(topic_types[topic]) for topic in required_topics}
    event_index = 0
    messages_written = 0
    point_values_rebased = 0
    point_values_preserved = 0
    output_start_ns: int | None = None
    output_end_ns: int | None = None
    previous_bag_ns: int | None = None

    try:
        while reader.has_next():
            topic, serialized, bag_ns_raw = reader.read_next()
            if topic not in required_topics:
                continue
            if event_index >= len(events):
                raise RuntimeError("input reader produced more required messages on write pass")
            event = events[event_index]
            if event.topic != topic or event.bag_ns != int(bag_ns_raw):
                raise RuntimeError("input reader order changed between analysis and write passes")

            offset_ns = offsets[event_index]
            msg = deserialize_message(serialized, msg_types[topic])
            input_header_ns = ns_from_msg_stamp(msg.header.stamp)
            if input_header_ns != event.header_ns:
                raise RuntimeError("input header timestamp changed between analysis and write passes")
            if topic == args.lidar_topic:
                rebased, preserved = rebase_point_timestamps(
                    msg, offset_ns, args.timestamp_field, args.max_point_header_offset_sec
                )
                point_values_rebased += rebased
                point_values_preserved += preserved
            set_msg_stamp(msg.header.stamp, input_header_ns - offset_ns)

            output_bag_ns = int(bag_ns_raw) - offset_ns
            if previous_bag_ns is not None and output_bag_ns < previous_bag_ns:
                raise RuntimeError("output rosbag timestamp order became non-monotonic")
            writer.write(topic, serialize_message(msg), output_bag_ns)
            previous_bag_ns = output_bag_ns
            output_start_ns = output_bag_ns if output_start_ns is None else output_start_ns
            output_end_ns = output_bag_ns
            messages_written += 1
            event_index += 1
    finally:
        del writer

    if event_index != len(events):
        raise RuntimeError("input reader produced fewer required messages on write pass")
    return {
        "messages_written": messages_written,
        "output_start_bag_ns": output_start_ns,
        "output_end_bag_ns": output_end_ns,
        "point_timestamp_values_rebased": point_values_rebased,
        "point_timestamp_values_preserved": point_values_preserved,
    }


def build_report(
    input_bag: Path,
    output_bag: Path,
    args: argparse.Namespace,
    collect_report: dict[str, Any],
    segments: list[Segment],
    write_report: dict[str, Any] | None,
) -> dict[str, Any]:
    total_offset_ns = segments[-1].cumulative_offset_ns
    return {
        "input_bag": str(input_bag),
        "output_bag": str(output_bag),
        "output_topics": [args.imu_topic, args.lidar_topic],
        "source_topics_omitted": True,
        "options": {
            "gap_threshold_sec": args.gap_threshold_sec,
            "bridge_gap_ms": args.bridge_gap_ms,
            "timestamp_field": args.timestamp_field,
            "max_point_header_offset_sec": args.max_point_header_offset_sec,
            "dry_run": args.dry_run,
        },
        **collect_report,
        "segments": [asdict(segment) for segment in segments],
        "segment_count": len(segments),
        "total_time_removed_ns": total_offset_ns,
        "total_time_removed_sec": total_offset_ns / NSEC_PER_SEC,
        "write": write_report,
    }


def print_report(report: dict[str, Any]) -> None:
    print(f"Input bag:  {report['input_bag']}")
    print(f"Output bag: {report['output_bag']}")
    print(f"Topics:     {', '.join(report['output_topics'])} (D-LIO input topics only)")
    print(
        "Policy:     "
        f"header gaps > {report['options']['gap_threshold_sec']:.3f}s, "
        f"bridge >= {report['options']['bridge_gap_ms']:.3f}ms"
    )
    print("\n[Segments]")
    print("  idx   messages        header range                   gap before   cumulative removed")
    for segment in report["segments"]:
        print(
            f"  {segment['index']:3d}  {segment['message_count']:9d}  "
            f"{fmt_sec(segment['first_header_ns'])} -> {fmt_sec(segment['last_header_ns'])}  "
            f"{fmt_ms(segment['gap_before_ns']):>12}  "
            f"{fmt_sec(segment['cumulative_offset_ns']):>18}"
        )
    print(f"\nTotal time removed: {fmt_sec(report['total_time_removed_ns'])}")
    if report["write"] is None:
        print("Dry run only: no output bag was written.")
        return
    write = report["write"]
    print("\n[Output]")
    print(f"  messages written:            {write['messages_written']}")
    print(
        "  output bag time:             "
        f"{fmt_sec(write['output_start_bag_ns'])} -> {fmt_sec(write['output_end_bag_ns'])}"
    )
    print(f"  per-point timestamps shifted: {write['point_timestamp_values_rebased']}")
    print(f"  invalid/sentinel values kept: {write['point_timestamp_values_preserved']}")


def main() -> int:
    args = parse_args()
    input_bag = Path(args.input_bag).expanduser().resolve()
    output_bag = (
        Path(args.output_bag).expanduser().resolve()
        if args.output_bag
        else default_output_path(input_bag).resolve()
    )
    try:
        validate_args(args)
        validate_paths(input_bag, output_bag, args.force, args.dry_run)
        events, collect_report, topic_types, output_topics = collect_events(
            input_bag, args.imu_topic, args.lidar_topic
        )
        segments = build_segments(
            events,
            int(args.gap_threshold_sec * NSEC_PER_SEC),
            int(args.bridge_gap_ms * 1_000_000.0),
        )
        offsets = offsets_for_events(events, segments)
        validate_output_order(events, offsets)
        write_report = None
        if not args.dry_run:
            write_report = write_rebased_bag(
                input_bag,
                output_bag,
                output_topics,
                topic_types,
                events,
                offsets,
                args,
            )
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    report = build_report(input_bag, output_bag, args, collect_report, segments, write_report)
    print_report(report)
    if args.json_path:
        json_path = Path(args.json_path).expanduser().resolve()
        with json_path.open("w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, sort_keys=True)
            f.write("\n")
        print(f"\nJSON report written: {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
