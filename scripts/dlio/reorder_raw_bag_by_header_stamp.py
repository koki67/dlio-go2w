#!/usr/bin/env python3
"""Rewrite a raw D-LIO rosbag in message header timestamp order.

This is an offline-only tool. It keeps message contents unchanged, but writes a
new rosbag where selected topics are ordered by header.stamp and the rosbag
record timestamp is also set to header.stamp.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import sqlite3
import sys
import yaml
from dataclasses import dataclass
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


@dataclass(frozen=True)
class MessageRef:
    db_path: Path
    row_id: int
    original_sequence: int
    topic: str
    input_bag_ns: int
    output_bag_ns: int
    stamp_source: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a new rosbag ordered by message header.stamp for raw D-LIO topics."
    )
    parser.add_argument("input_bag", help="Input rosbag2 directory containing metadata.yaml")
    parser.add_argument(
        "output_bag",
        nargs="?",
        help="Output rosbag2 directory. Default: <input_bag>_reordered",
    )
    parser.add_argument("--imu-topic", default="/go2w/imu", help="IMU topic ordered by header.stamp")
    parser.add_argument("--lidar-topic", default="/points_raw", help="LiDAR topic ordered by header.stamp")
    parser.add_argument(
        "--preserve-record-time-for-other-topics",
        action="store_true",
        help="Use original rosbag timestamps for topics other than IMU/LiDAR.",
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


def fmt_sec(ns: int | None) -> str:
    if ns is None:
        return "n/a"
    return f"{ns / NSEC_PER_SEC:.9f}s"


def fmt_ms(ns: int | None) -> str:
    if ns is None:
        return "n/a"
    return f"{ns / 1_000_000.0:.3f}ms"


def natural_key(path: Path) -> list[Any]:
    return [int(part) if part.isdigit() else part for part in re.split(r"(\d+)", path.name)]


def default_output_path(input_bag: Path) -> Path:
    return input_bag.with_name(f"{input_bag.name}_reordered")


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


def db_paths(input_bag: Path) -> list[Path]:
    paths = sorted(input_bag.glob("*.db3"), key=natural_key)
    if not paths:
        raise RuntimeError(f"no sqlite3 .db3 files found in input bag: {input_bag}")
    return paths


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


def read_topics(input_bag: Path) -> tuple[list[Any], dict[str, str]]:
    reader = open_reader(input_bag)
    topics = reader.get_all_topics_and_types()
    topic_types = {topic.name: topic.type for topic in topics}
    return topics, topic_types


def collect_message_refs(
    input_bag: Path,
    topic_types: dict[str, str],
    stamp_topics: set[str],
    preserve_other_record_time: bool,
) -> tuple[list[MessageRef], dict[str, Any]]:
    type_cache = {topic: get_message(type_name) for topic, type_name in topic_types.items()}
    refs: list[MessageRef] = []
    sequence = 0
    per_topic_last_input_header: dict[str, int] = {}
    per_topic_input_non_increasing: dict[str, int] = {}
    per_topic_counts: dict[str, int] = {}
    top_input_delays: list[dict[str, Any]] = []

    for db_path in db_paths(input_bag):
        conn = sqlite3.connect(str(db_path))
        try:
            topic_ids = {
                int(row[0]): str(row[1])
                for row in conn.execute("SELECT id, name FROM topics")
            }
            rows = conn.execute(
                "SELECT id, topic_id, timestamp, data FROM messages ORDER BY timestamp, id"
            )
            for row_id, topic_id, input_bag_ns, data in rows:
                topic = topic_ids[int(topic_id)]
                per_topic_counts[topic] = per_topic_counts.get(topic, 0) + 1
                stamp_source = "bag_time"
                output_bag_ns = int(input_bag_ns)

                if topic in stamp_topics:
                    msg = deserialize_message(bytes(data), type_cache[topic])
                    if not hasattr(msg, "header") or not hasattr(msg.header, "stamp"):
                        raise RuntimeError(f"topic has no header.stamp: {topic}")
                    header_ns = ns_from_msg_stamp(msg.header.stamp)
                    output_bag_ns = header_ns
                    stamp_source = "header.stamp"

                    last_header = per_topic_last_input_header.get(topic)
                    if last_header is not None and header_ns <= last_header:
                        per_topic_input_non_increasing[topic] = (
                            per_topic_input_non_increasing.get(topic, 0) + 1
                        )
                    per_topic_last_input_header[topic] = header_ns

                    delay_ns = int(input_bag_ns) - header_ns
                    top_input_delays.append(
                        {
                            "topic": topic,
                            "input_sequence": sequence,
                            "input_bag_ns": int(input_bag_ns),
                            "header_ns": header_ns,
                            "delay_ns": delay_ns,
                        }
                    )
                elif not preserve_other_record_time:
                    stamp_source = "bag_time"

                refs.append(
                    MessageRef(
                        db_path=db_path,
                        row_id=int(row_id),
                        original_sequence=sequence,
                        topic=topic,
                        input_bag_ns=int(input_bag_ns),
                        output_bag_ns=output_bag_ns,
                        stamp_source=stamp_source,
                    )
                )
                sequence += 1
        finally:
            conn.close()

    top_input_delays.sort(key=lambda item: item["delay_ns"], reverse=True)
    report = {
        "messages_read": len(refs),
        "per_topic_counts": per_topic_counts,
        "per_topic_input_non_increasing": per_topic_input_non_increasing,
        "top_input_header_delays": top_input_delays[:10],
    }
    return refs, report


def write_reordered_bag(output_bag: Path, topics: list[Any], refs: list[MessageRef]) -> dict[str, Any]:
    writer = open_writer(output_bag, topics)
    sorted_refs = sorted(refs, key=lambda ref: (ref.output_bag_ns, ref.original_sequence))
    conn_cache: dict[Path, sqlite3.Connection] = {}
    moved_messages = 0

    try:
        for output_sequence, ref in enumerate(sorted_refs):
            if output_sequence != ref.original_sequence:
                moved_messages += 1

            conn = conn_cache.get(ref.db_path)
            if conn is None:
                conn = sqlite3.connect(str(ref.db_path))
                conn_cache[ref.db_path] = conn

            row = conn.execute("SELECT data FROM messages WHERE id = ?", (ref.row_id,)).fetchone()
            if row is None:
                raise RuntimeError(f"message row not found: {ref.db_path}:{ref.row_id}")
            writer.write(ref.topic, bytes(row[0]), ref.output_bag_ns)
    finally:
        for conn in conn_cache.values():
            conn.close()

    return {
        "messages_written": len(sorted_refs),
        "messages_moved_in_output_order": moved_messages,
        "output_start_ns": sorted_refs[0].output_bag_ns if sorted_refs else None,
        "output_end_ns": sorted_refs[-1].output_bag_ns if sorted_refs else None,
    }


def load_metadata(bag_path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    metadata_path = bag_path / "metadata.yaml"
    with metadata_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    info = data.get("rosbag2_bagfile_information")
    if not isinstance(info, dict):
        raise RuntimeError(f"invalid rosbag2 metadata: {metadata_path}")
    return data, info


def topic_metadata_by_name(info: dict[str, Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for entry in info.get("topics_with_message_count", []):
        metadata = entry.get("topic_metadata") or {}
        name = metadata.get("name")
        if name:
            result[str(name)] = metadata
    return result


def qos_profiles_as_humble_string(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, str):
        return value
    return yaml.safe_dump(value, sort_keys=False).strip()


def count_output_messages(output_bag: Path) -> tuple[dict[str, int], dict[str, int]]:
    per_file: dict[str, int] = {}
    per_topic: dict[str, int] = {}

    for db_path in sorted(output_bag.glob("*.db3"), key=natural_key):
        conn = sqlite3.connect(str(db_path))
        try:
            per_file[db_path.name] = int(
                conn.execute("SELECT count(*) FROM messages").fetchone()[0]
            )
            rows = conn.execute(
                "SELECT topics.name, count(*) "
                "FROM messages JOIN topics ON topics.id = messages.topic_id "
                "GROUP BY topics.name"
            )
            for topic, count in rows:
                per_topic[str(topic)] = int(count)
        finally:
            conn.close()

    return per_file, per_topic


def normalize_output_metadata(input_bag: Path, output_bag: Path) -> dict[str, Any]:
    input_data, input_info = load_metadata(input_bag)
    output_data, output_info = load_metadata(output_bag)

    input_topics = topic_metadata_by_name(input_info)
    per_file_count, per_topic_count = count_output_messages(output_bag)

    if "version" in input_info:
        output_info["version"] = input_info["version"]

    for optional_key in ("custom_data", "ros_distro"):
        if optional_key not in input_info:
            output_info.pop(optional_key, None)

    output_info["message_count"] = sum(per_file_count.values())

    for entry in output_info.get("topics_with_message_count", []):
        metadata = entry.get("topic_metadata") or {}
        topic_name = metadata.get("name")
        if topic_name in per_topic_count:
            entry["message_count"] = per_topic_count[topic_name]

        input_metadata = input_topics.get(str(topic_name))
        if not input_metadata:
            continue

        metadata["offered_qos_profiles"] = qos_profiles_as_humble_string(
            input_metadata.get("offered_qos_profiles", "")
        )
        if "type_description_hash" not in input_metadata:
            metadata.pop("type_description_hash", None)

    for entry in output_info.get("files", []):
        path = entry.get("path")
        if path in per_file_count:
            entry["message_count"] = per_file_count[path]

    with (output_bag / "metadata.yaml").open("w", encoding="utf-8") as f:
        yaml.safe_dump(output_data, f, sort_keys=False)

    return {
        "metadata_normalized": True,
        "metadata_version": output_info.get("version"),
    }


def print_report(report: dict[str, Any]) -> None:
    print(f"Input bag:  {report['input_bag']}")
    print(f"Output bag: {report['output_bag']}")
    print(f"Stamp topics: {', '.join(report['stamp_topics'])}")

    print("\n[Summary]")
    print(f"  messages read:    {report['messages_read']}")
    print(f"  messages written: {report['messages_written']}")
    print(f"  moved in order:   {report['messages_moved_in_output_order']}")
    print(f"  output range:     {fmt_sec(report['output_start_ns'])} -> {fmt_sec(report['output_end_ns'])}")

    print("\n[Per-topic counts]")
    for topic, count in sorted(report["per_topic_counts"].items()):
        non_inc = report["per_topic_input_non_increasing"].get(topic, 0)
        print(f"  {topic}: {count} messages, input non-increasing={non_inc}")

    print("\n[Top input bag_time - header.stamp delays]")
    if not report["top_input_header_delays"]:
        print("  none")
        return
    print("   input_seq  topic           header_stamp          input_bag_time      delay")
    for item in report["top_input_header_delays"]:
        print(
            f"  {item['input_sequence']:10d}  "
            f"{item['topic']:<14}  "
            f"{fmt_sec(item['header_ns']):>20}  "
            f"{fmt_sec(item['input_bag_ns']):>20}  "
            f"{fmt_ms(item['delay_ns']):>12}"
        )


def main() -> int:
    args = parse_args()
    input_bag = Path(args.input_bag).expanduser().resolve()
    output_bag = (
        Path(args.output_bag).expanduser().resolve()
        if args.output_bag
        else default_output_path(input_bag).resolve()
    )

    stamp_topics = {args.imu_topic, args.lidar_topic}

    try:
        validate_paths(input_bag, output_bag, args.force)
        topics, topic_types = read_topics(input_bag)
        missing = [topic for topic in sorted(stamp_topics) if topic not in topic_types]
        if missing:
            found = "\n".join(f"  {name}: {type_name}" for name, type_name in sorted(topic_types.items()))
            raise RuntimeError(f"missing required topic(s): {', '.join(missing)}\nTopics found:\n{found}")

        refs, collect_report = collect_message_refs(
            input_bag,
            topic_types,
            stamp_topics,
            args.preserve_record_time_for_other_topics,
        )
        write_report = write_reordered_bag(output_bag, topics, refs)
        metadata_report = normalize_output_metadata(input_bag, output_bag)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    report = {
        "input_bag": str(input_bag),
        "output_bag": str(output_bag),
        "stamp_topics": sorted(stamp_topics),
        **collect_report,
        **write_report,
        **metadata_report,
    }
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
