#!/usr/bin/env python3
"""Write a cleaned raw D-LIO rosbag by dropping bad LiDAR timestamp frames.

The original bag is never modified. By default this script removes only LiDAR
PointCloud2 messages whose header.stamp is non-increasing relative to the last
kept LiDAR frame. This is intended for recorded GO2-W + Hesai raw bags where a
single delayed/out-of-order point cloud can break offline D-LIO reconstruction.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path
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
class BagInfo:
    topic_types: dict[str, str]
    topics: list[Any]
    bag_start_ns: int | None = None
    imu_min_ns: int | None = None
    imu_max_ns: int | None = None
    imu_count: int = 0


@dataclass
class DroppedFrame:
    index: int
    header_ns: int
    bag_ns: int
    prev_kept_header_ns: int | None
    record_delay_ns: int
    reasons: list[str] = field(default_factory=list)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a cleaned raw D-LIO rosbag by dropping anomalous LiDAR frames."
    )
    parser.add_argument("input_bag", help="Input rosbag2 directory containing metadata.yaml")
    parser.add_argument(
        "output_bag",
        nargs="?",
        help="Output rosbag2 directory. Default: <input_bag>_clean",
    )
    parser.add_argument("--lidar-topic", default="/points_raw", help="LiDAR PointCloud2 topic")
    parser.add_argument("--imu-topic", default="/go2w/imu", help="IMU topic used for optional range checks")
    parser.add_argument(
        "--trim-start-sec",
        type=float,
        default=0.0,
        help="Drop all messages whose bag record time is within this many seconds from bag start.",
    )
    parser.add_argument(
        "--drop-record-delay-ms",
        type=float,
        default=0.0,
        help="Also drop LiDAR frames where bag_time - header.stamp exceeds this threshold. 0 disables it.",
    )
    parser.add_argument(
        "--drop-lidar-outside-imu-range",
        action="store_true",
        help="Drop LiDAR frames whose header.stamp is outside the IMU header.stamp range.",
    )
    parser.add_argument(
        "--keep-non-increasing",
        action="store_true",
        help="Do not drop non-increasing LiDAR header.stamp frames.",
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
        help="Optional path to write a machine-readable cleaning report.",
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


def default_output_path(input_bag: Path) -> Path:
    return input_bag.with_name(f"{input_bag.name}_clean")


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def open_writer(output_path: Path, topics: list[Any]) -> rosbag2_py.SequentialWriter:
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=str(output_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    writer.open(storage_options, converter_options)
    for topic in topics:
        writer.create_topic(topic)
    return writer


def validate_paths(input_bag: Path, output_bag: Path, force: bool) -> None:
    if not input_bag.is_dir():
        raise RuntimeError(f"input bag directory not found: {input_bag}")
    if not (input_bag / "metadata.yaml").is_file():
        raise RuntimeError(f"metadata.yaml not found in input bag directory: {input_bag}")
    if output_bag.resolve() == input_bag.resolve():
        raise RuntimeError("output bag must be different from input bag")
    if output_bag.exists():
        if not force:
            raise RuntimeError(f"output path already exists: {output_bag} (use --force to replace it)")
        shutil.rmtree(output_bag)


def inspect_bag(input_bag: Path, lidar_topic: str, imu_topic: str) -> BagInfo:
    reader = open_reader(input_bag)
    topics = reader.get_all_topics_and_types()
    topic_types = {topic.name: topic.type for topic in topics}
    if lidar_topic not in topic_types:
        found = "\n".join(f"  {name}: {type_name}" for name, type_name in sorted(topic_types.items()))
        raise RuntimeError(f"missing LiDAR topic: {lidar_topic}\nTopics found:\n{found}")

    info = BagInfo(topic_types=topic_types, topics=topics)
    imu_msg_type = get_message(topic_types[imu_topic]) if imu_topic in topic_types else None

    while reader.has_next():
        topic, serialized, bag_stamp_ns = reader.read_next()
        if info.bag_start_ns is None:
            info.bag_start_ns = int(bag_stamp_ns)

        if imu_msg_type is not None and topic == imu_topic:
            msg = deserialize_message(serialized, imu_msg_type)
            if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
                stamp_ns = ns_from_msg_stamp(msg.header.stamp)
                info.imu_count += 1
                info.imu_min_ns = stamp_ns if info.imu_min_ns is None else min(info.imu_min_ns, stamp_ns)
                info.imu_max_ns = stamp_ns if info.imu_max_ns is None else max(info.imu_max_ns, stamp_ns)

    if info.bag_start_ns is None:
        raise RuntimeError(f"input bag has no messages: {input_bag}")
    return info


def should_drop_lidar(
    header_ns: int,
    bag_ns: int,
    last_kept_header_ns: int | None,
    info: BagInfo,
    args: argparse.Namespace,
) -> list[str]:
    reasons: list[str] = []

    if not args.keep_non_increasing and last_kept_header_ns is not None:
        if header_ns <= last_kept_header_ns:
            reasons.append("non_increasing")

    if args.drop_record_delay_ms > 0.0:
        threshold_ns = int(args.drop_record_delay_ms * 1_000_000.0)
        if bag_ns - header_ns > threshold_ns:
            reasons.append("record_delay")

    if args.drop_lidar_outside_imu_range:
        if info.imu_min_ns is None or info.imu_max_ns is None:
            reasons.append("imu_range_unavailable")
        elif header_ns < info.imu_min_ns:
            reasons.append("before_imu_range")
        elif header_ns > info.imu_max_ns:
            reasons.append("after_imu_range")

    return reasons


def clean_bag(input_bag: Path, output_bag: Path, info: BagInfo, args: argparse.Namespace) -> dict[str, Any]:
    reader = open_reader(input_bag)
    writer = open_writer(output_bag, info.topics)
    lidar_msg_type = get_message(info.topic_types[args.lidar_topic])

    trim_before_ns = None
    if args.trim_start_sec > 0.0:
        trim_before_ns = int(info.bag_start_ns + args.trim_start_sec * NSEC_PER_SEC)

    messages_read = 0
    messages_written = 0
    messages_trimmed = 0
    lidar_seen = 0
    lidar_written = 0
    last_kept_lidar_header_ns: int | None = None
    dropped_lidar: list[DroppedFrame] = []

    while reader.has_next():
        topic, serialized, bag_stamp_ns_raw = reader.read_next()
        bag_ns = int(bag_stamp_ns_raw)
        messages_read += 1

        if trim_before_ns is not None and bag_ns < trim_before_ns:
            messages_trimmed += 1
            continue

        if topic != args.lidar_topic:
            writer.write(topic, serialized, bag_ns)
            messages_written += 1
            continue

        lidar_seen += 1
        msg = deserialize_message(serialized, lidar_msg_type)
        if not hasattr(msg, "header") or not hasattr(msg.header, "stamp"):
            raise RuntimeError(f"LiDAR topic has no header.stamp: {args.lidar_topic}")
        header_ns = ns_from_msg_stamp(msg.header.stamp)

        reasons = should_drop_lidar(header_ns, bag_ns, last_kept_lidar_header_ns, info, args)
        if reasons:
            dropped_lidar.append(
                DroppedFrame(
                    index=lidar_seen,
                    header_ns=header_ns,
                    bag_ns=bag_ns,
                    prev_kept_header_ns=last_kept_lidar_header_ns,
                    record_delay_ns=bag_ns - header_ns,
                    reasons=reasons,
                )
            )
            continue

        writer.write(topic, serialized, bag_ns)
        messages_written += 1
        lidar_written += 1
        last_kept_lidar_header_ns = header_ns

    return {
        "input_bag": str(input_bag),
        "output_bag": str(output_bag),
        "lidar_topic": args.lidar_topic,
        "imu_topic": args.imu_topic,
        "messages_read": messages_read,
        "messages_written": messages_written,
        "messages_trimmed_by_bag_start": messages_trimmed,
        "lidar_frames_seen": lidar_seen,
        "lidar_frames_written": lidar_written,
        "lidar_frames_dropped": len(dropped_lidar),
        "imu_header_range": {
            "sample_count": info.imu_count,
            "start_ns": info.imu_min_ns,
            "start_sec": None if info.imu_min_ns is None else info.imu_min_ns / NSEC_PER_SEC,
            "end_ns": info.imu_max_ns,
            "end_sec": None if info.imu_max_ns is None else info.imu_max_ns / NSEC_PER_SEC,
        },
        "options": {
            "trim_start_sec": args.trim_start_sec,
            "drop_record_delay_ms": args.drop_record_delay_ms,
            "drop_lidar_outside_imu_range": args.drop_lidar_outside_imu_range,
            "keep_non_increasing": args.keep_non_increasing,
        },
        "dropped_lidar_frames": [
            {
                "index": frame.index,
                "header_stamp_ns": frame.header_ns,
                "header_stamp_sec": frame.header_ns / NSEC_PER_SEC,
                "bag_time_ns": frame.bag_ns,
                "bag_time_sec": frame.bag_ns / NSEC_PER_SEC,
                "prev_kept_header_stamp_ns": frame.prev_kept_header_ns,
                "prev_kept_header_stamp_sec": None
                if frame.prev_kept_header_ns is None
                else frame.prev_kept_header_ns / NSEC_PER_SEC,
                "record_delay_ns": frame.record_delay_ns,
                "record_delay_ms": frame.record_delay_ns / 1_000_000.0,
                "reasons": frame.reasons,
            }
            for frame in dropped_lidar
        ],
    }


def print_report(report: dict[str, Any]) -> None:
    print(f"Input bag:  {report['input_bag']}")
    print(f"Output bag: {report['output_bag']}")
    print(f"LiDAR:      {report['lidar_topic']}")
    print(f"IMU:        {report['imu_topic']}")

    print("\n[Summary]")
    print(f"  messages read:    {report['messages_read']}")
    print(f"  messages written: {report['messages_written']}")
    print(f"  messages trimmed: {report['messages_trimmed_by_bag_start']}")
    print(f"  lidar seen:       {report['lidar_frames_seen']}")
    print(f"  lidar written:    {report['lidar_frames_written']}")
    print(f"  lidar dropped:    {report['lidar_frames_dropped']}")

    imu = report["imu_header_range"]
    print("\n[IMU header range]")
    print(f"  samples: {imu['sample_count']}")
    print(f"  range:   {fmt_sec(imu['start_ns'])} -> {fmt_sec(imu['end_ns'])}")

    print("\n[Dropped LiDAR frames]")
    if not report["dropped_lidar_frames"]:
        print("  none")
        return

    print("   index          header_stamp    prev_kept_header      bag_time          delay  reasons")
    for frame in report["dropped_lidar_frames"]:
        reasons = ",".join(frame["reasons"])
        print(
            f"  {frame['index']:6d}  "
            f"{fmt_sec(frame['header_stamp_ns']):>20}  "
            f"{fmt_sec(frame['prev_kept_header_stamp_ns']):>20}  "
            f"{fmt_sec(frame['bag_time_ns']):>20}  "
            f"{fmt_ms(frame['record_delay_ns']):>12}  "
            f"{reasons}"
        )


def main() -> int:
    args = parse_args()
    input_bag = Path(args.input_bag).expanduser().resolve()
    output_bag = (
        Path(args.output_bag).expanduser().resolve()
        if args.output_bag
        else default_output_path(input_bag).resolve()
    )

    try:
        validate_paths(input_bag, output_bag, args.force)
        info = inspect_bag(input_bag, args.lidar_topic, args.imu_topic)
        if args.drop_lidar_outside_imu_range and info.imu_count == 0:
            raise RuntimeError(
                f"--drop-lidar-outside-imu-range requested, but no IMU stamps were found on {args.imu_topic}"
            )
        report = clean_bag(input_bag, output_bag, info, args)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print_report(report)

    if args.json_path:
        out = Path(args.json_path).expanduser().resolve()
        with out.open("w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, sort_keys=True)
            f.write("\n")
        print(f"\nJSON report written: {out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
