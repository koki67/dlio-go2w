#!/usr/bin/env python3
"""Sample CPU time and RSS for named processes from Linux /proc.

Targets are rooted at a process started by the experiment runner. A selector
then identifies the actual worker among that root and its descendants rather
than attributing the wrapper ``ros2 launch`` process.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import signal
import threading
import time
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Target:
    label: str
    root_pid: int
    selector: str


@dataclass
class ProcessSample:
    pid: int
    parent_pid: int
    command: str
    cpu_time_s: float
    rss_kib: int


@dataclass
class TargetSummary:
    pids: set[int] = field(default_factory=set)
    first_cpu_time: dict[int, float] = field(default_factory=dict)
    last_cpu_time: dict[int, float] = field(default_factory=dict)
    first_wall_time_s: float | None = None
    last_wall_time_s: float | None = None
    peak_rss_kib: int = 0
    samples: int = 0


def _parse_target(value: str) -> Target:
    try:
        label, root_pid, selector = value.split(":", 2)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "target must have the form LABEL:ROOT_PID:COMMAND_SUBSTRING"
        ) from exc
    if not label or not selector:
        raise argparse.ArgumentTypeError("target label and selector cannot be empty")
    try:
        pid = int(root_pid)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("target root PID must be an integer") from exc
    if pid <= 0:
        raise argparse.ArgumentTypeError("target root PID must be positive")
    return Target(label=label, root_pid=pid, selector=selector)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target",
        action="append",
        required=True,
        type=_parse_target,
        help="LABEL:ROOT_PID:COMMAND_SUBSTRING; use '*' to select the whole tree",
    )
    parser.add_argument("--output", required=True, type=Path, help="Long-form CSV path")
    parser.add_argument(
        "--summary",
        required=True,
        type=Path,
        help="JSON summary written when sampling stops",
    )
    parser.add_argument("--interval", type=float, default=1.0, help="Sampling interval in seconds")
    parser.add_argument(
        "--stop-file",
        type=Path,
        help="Stop cleanly when this path appears",
    )
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error("--interval must be greater than zero")
    labels = [target.label for target in args.target]
    if len(labels) != len(set(labels)):
        parser.error("target labels must be unique")
    return args


def _read_process(pid: int, clock_ticks: int) -> ProcessSample | None:
    proc = Path("/proc") / str(pid)
    try:
        stat = (proc / "stat").read_text(encoding="utf-8")
        # The command in parentheses can contain spaces and parentheses. The
        # fields following the final ') ' have stable positions.
        fields = stat[stat.rfind(") ") + 2:].split()
        parent_pid = int(fields[1])
        cpu_time_s = (int(fields[11]) + int(fields[12])) / clock_ticks

        rss_kib = 0
        for line in (proc / "status").read_text(encoding="utf-8").splitlines():
            if line.startswith("VmRSS:"):
                rss_kib = int(line.split()[1])
                break

        command_bytes = (proc / "cmdline").read_bytes()
        command = command_bytes.replace(b"\0", b" ").decode("utf-8", errors="replace").strip()
        if not command:
            command = stat[stat.find("(") + 1:stat.rfind(")")]
    except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError, IndexError):
        return None

    return ProcessSample(
        pid=pid,
        parent_pid=parent_pid,
        command=command,
        cpu_time_s=cpu_time_s,
        rss_kib=rss_kib,
    )


def _process_table(clock_ticks: int) -> dict[int, ProcessSample]:
    processes: dict[int, ProcessSample] = {}
    try:
        entries = os.scandir("/proc")
    except OSError:
        return processes
    with entries:
        for entry in entries:
            if not entry.name.isdigit():
                continue
            sample = _read_process(int(entry.name), clock_ticks)
            if sample is not None:
                processes[sample.pid] = sample
    return processes


def _tree_pids(root_pid: int, processes: dict[int, ProcessSample]) -> set[int]:
    children: dict[int, list[int]] = {}
    for sample in processes.values():
        children.setdefault(sample.parent_pid, []).append(sample.pid)

    found: set[int] = set()
    pending = [root_pid]
    while pending:
        pid = pending.pop()
        if pid in found or pid not in processes:
            continue
        found.add(pid)
        pending.extend(children.get(pid, ()))
    return found


def _write_summary(
    path: Path,
    interval_s: float,
    elapsed_wall_time_s: float,
    summaries: dict[str, TargetSummary],
) -> None:
    processes = {}
    for label, summary in summaries.items():
        cpu_delta = sum(
            max(0.0, summary.last_cpu_time[pid] - summary.first_cpu_time[pid])
            for pid in summary.last_cpu_time
            if pid in summary.first_cpu_time
        )
        if summary.first_wall_time_s is None or summary.last_wall_time_s is None:
            observed_wall = 0.0
        else:
            observed_wall = max(0.0, summary.last_wall_time_s - summary.first_wall_time_s)
        processes[label] = {
            "pids": sorted(summary.pids),
            "samples": summary.samples,
            "observed_wall_time_s": round(observed_wall, 6),
            "cpu_time_delta_s": round(cpu_delta, 6),
            "average_cpu_cores": round(cpu_delta / observed_wall, 6) if observed_wall else 0.0,
            "peak_rss_kib": summary.peak_rss_kib,
        }

    document = {
        "interval_s": interval_s,
        "elapsed_wall_time_s": round(elapsed_wall_time_s, 6),
        "processes": processes,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    temporary_path.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    temporary_path.replace(path)


def main() -> int:
    args = _arguments()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)

    stop_event = threading.Event()

    def request_stop(_signum=None, _frame=None):
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    summaries = {target.label: TargetSummary() for target in args.target}
    start = time.monotonic()

    with args.output.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(["wall_time_s", "pid", "process", "cpu_time_s", "rss_kib"])

        while not stop_event.is_set():
            wall_time_s = time.monotonic() - start
            processes = _process_table(clock_ticks)

            for target in args.target:
                tree = _tree_pids(target.root_pid, processes)
                selected = [
                    processes[pid]
                    for pid in sorted(tree)
                    if target.selector == "*" or target.selector in processes[pid].command
                ]
                aggregate_rss_kib = sum(sample.rss_kib for sample in selected)
                summary = summaries[target.label]
                if selected:
                    summary.peak_rss_kib = max(summary.peak_rss_kib, aggregate_rss_kib)
                    if summary.first_wall_time_s is None:
                        summary.first_wall_time_s = wall_time_s
                    summary.last_wall_time_s = wall_time_s

                for sample in selected:
                    writer.writerow(
                        [
                            f"{wall_time_s:.6f}",
                            sample.pid,
                            target.label,
                            f"{sample.cpu_time_s:.6f}",
                            sample.rss_kib,
                        ]
                    )
                    summary.pids.add(sample.pid)
                    summary.samples += 1
                    summary.first_cpu_time.setdefault(sample.pid, sample.cpu_time_s)
                    summary.last_cpu_time[sample.pid] = sample.cpu_time_s

            output.flush()
            if args.stop_file is not None and args.stop_file.exists():
                break
            stop_event.wait(args.interval)

    elapsed_wall_time_s = time.monotonic() - start
    _write_summary(args.summary, args.interval, elapsed_wall_time_s, summaries)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
