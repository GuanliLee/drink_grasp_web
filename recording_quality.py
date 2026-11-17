from __future__ import annotations

import bisect
import json
import re
import struct
from pathlib import Path
from typing import Any, Iterable

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


CAMERA_TOPICS = (
    "/cam_high/color/image_raw",
    "/cam_left/color/image_raw",
    "/cam_right/color/image_raw",
)
FEEDBACK_TOPICS = (
    "/joint_states_left",
    "/joint_states_right",
)
ACTION_TOPICS = (
    "/joint_ctrl_cmd_left",
    "/joint_ctrl_cmd_right",
)
EVENT_TOPIC = "/act/episode_event"


def _cdr_header_stamp_ns(serialized: bytes, fallback_ns: int) -> int:
    """Read std_msgs/Header.stamp without decoding the large Image payload."""
    del fallback_ns
    if len(serialized) < 12:
        return 0
    little_endian = bool(serialized[1] & 0x01)
    byte_order = "<" if little_endian else ">"
    try:
        seconds, nanoseconds = struct.unpack_from(f"{byte_order}iI", serialized, 4)
    except struct.error:
        return 0
    timestamp_ns = int(seconds) * 1_000_000_000 + int(nanoseconds)
    return timestamp_ns if timestamp_ns > 0 else 0


def _nearest_delta_ms(sorted_times: list[int], target_ns: int) -> float:
    position = bisect.bisect_left(sorted_times, target_ns)
    candidates: list[int] = []
    if position < len(sorted_times):
        candidates.append(sorted_times[position])
    if position > 0:
        candidates.append(sorted_times[position - 1])
    if not candidates:
        return float("inf")
    return min(abs(item - target_ns) for item in candidates) / 1e6


def _camera_timing_metrics(
    stream_times: Iterable[int],
    *,
    start_ns: int,
    end_ns: int,
    target_times: Iterable[int],
) -> dict[str, int | float]:
    timeline = sorted(stamp for stamp in stream_times if stamp > 0)
    selected = [stamp for stamp in timeline if start_ns <= stamp <= end_ns]
    duration = (selected[-1] - selected[0]) / 1e9 if len(selected) >= 2 else 0.0
    rate_hz = (len(selected) - 1) / duration if duration > 0.0 else 0.0
    gaps_ms = [
        (right - left) / 1e6
        for left, right in zip(selected, selected[1:])
    ]

    # The recorder starts before START and stops after END. Keep rate/gap statistics
    # inside the episode, but let boundary targets use the nearest captured frame on
    # either side so an arbitrary START/END phase does not create a false sync miss.
    max_delta = max(
        (_nearest_delta_ms(timeline, target) for target in target_times),
        default=float("inf"),
    )
    return {
        "frames": len(selected),
        "rate_hz": rate_hz,
        "max_gap_ms": max(gaps_ms) if gaps_ms else float("inf"),
        "max_delta_ms": max_delta,
    }


def _transport_loss_count(log_file: Path | None) -> int | None:
    if log_file is None or not log_file.is_file():
        return None
    text = log_file.read_text(encoding="utf-8", errors="replace")
    matches = re.findall(
        r"Number of messages lost on (?:the )?transport layer:\s*(\d+)",
        text,
    )
    return int(matches[-1]) if matches else None


def _active_arms(start_event: dict[str, Any]) -> tuple[str, ...]:
    configured = str(start_event.get("arm") or "right").strip().lower()
    if configured in {"left_right", "right_left", "both", "dual"}:
        return ("left", "right")
    trigger_arm = str(start_event.get("trigger_arm") or configured).strip().lower()
    return (trigger_arm,) if trigger_arm in {"left", "right"} else ("right",)


def validate_recording(
    bag_dir: Path,
    *,
    log_file: Path | None = None,
    camera_topics: Iterable[str] = CAMERA_TOPICS,
    feedback_topics: Iterable[str] = FEEDBACK_TOPICS,
    action_topics: Iterable[str] = ACTION_TOPICS,
    event_topic: str = EVENT_TOPIC,
    min_camera_hz: float = 29.0,
    max_camera_gap_ms: float = 50.0,
    control_hz: float = 30.0,
    max_camera_delta_ms: float = 25.0,
) -> dict[str, Any]:
    bag_dir = Path(bag_dir)
    failures: list[str] = []
    warnings: list[str] = []
    streams: dict[str, list[int]] = {
        topic: []
        for topic in (*tuple(camera_topics), *tuple(feedback_topics), *tuple(action_topics))
    }
    events: list[dict[str, Any]] = []

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    available_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    required_topics = set(streams) | {event_topic}
    missing_topics = sorted(required_topics - set(available_types))
    for topic in missing_topics:
        failures.append(f"missing_topic:{topic}")

    while reader.has_next():
        topic, serialized, bag_timestamp_ns = reader.read_next()
        if topic in streams:
            streams[topic].append(
                _cdr_header_stamp_ns(serialized, int(bag_timestamp_ns))
            )
        elif topic == event_topic:
            try:
                message = deserialize_message(serialized, String)
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    raise ValueError("event payload is not an object")
                payload["_bag_timestamp_ns"] = int(bag_timestamp_ns)
                events.append(payload)
            except Exception as exc:  # noqa: BLE001
                failures.append(f"invalid_episode_event:{exc}")

    start_events = [item for item in events if item.get("event") == "START"]
    end_events = [item for item in events if item.get("event") == "END"]
    abort_events = [item for item in events if item.get("event") == "ABORT"]
    if len(start_events) != 1:
        failures.append(f"start_event_count:{len(start_events)}")
    if len(end_events) != 1:
        failures.append(f"end_event_count:{len(end_events)}")
    if abort_events:
        failures.append(f"abort_event_count:{len(abort_events)}")

    start_ns = 0
    end_ns = 0
    start_event: dict[str, Any] = {}
    if len(start_events) == 1 and len(end_events) == 1:
        start_event = start_events[0]
        start_ns = int(start_event.get("stamp_ns") or start_event["_bag_timestamp_ns"])
        end_ns = int(end_events[0].get("stamp_ns") or end_events[0]["_bag_timestamp_ns"])
        if end_ns <= start_ns:
            failures.append("episode_interval_not_positive")

    camera_report: dict[str, Any] = {}
    if start_ns > 0 and end_ns > start_ns:
        period_ns = int(round(1e9 / control_hz))
        target_times = list(range(start_ns, end_ns + 1, period_ns))
        for topic in camera_topics:
            metrics = _camera_timing_metrics(
                streams.get(topic, []),
                start_ns=start_ns,
                end_ns=end_ns,
                target_times=target_times,
            )
            frames = int(metrics["frames"])
            rate_hz = float(metrics["rate_hz"])
            max_gap = float(metrics["max_gap_ms"])
            max_delta = float(metrics["max_delta_ms"])
            camera_report[topic] = {
                "frames": frames,
                "rate_hz": round(rate_hz, 3),
                "max_gap_ms": round(max_gap, 3),
                "max_control_delta_ms": round(max_delta, 3),
            }
            if frames < 2:
                failures.append(f"camera_no_interval:{topic}")
            elif rate_hz < min_camera_hz:
                warnings.append(f"camera_rate:{topic}:{rate_hz:.3f}")
            if max_gap > max_camera_gap_ms:
                warnings.append(f"camera_gap:{topic}:{max_gap:.3f}ms")
            if max_delta > max_camera_delta_ms:
                warnings.append(f"camera_sync:{topic}:{max_delta:.3f}ms")

        for topic in feedback_topics:
            values = sorted(streams.get(topic, []))
            if not values:
                failures.append(f"feedback_empty:{topic}")
                continue
            if _nearest_delta_ms(values, start_ns) > 100.0:
                failures.append(f"feedback_start_not_covered:{topic}")
            if _nearest_delta_ms(values, end_ns) > 100.0:
                failures.append(f"feedback_end_not_covered:{topic}")

        active_arms = set(_active_arms(start_event))
        for topic in action_topics:
            arm = "left" if topic.endswith("_left") else "right"
            values = sorted(streams.get(topic, []))
            interval_values = [value for value in values if start_ns <= value <= end_ns]
            covered_at_start = bool(values and values[0] <= start_ns <= values[-1])
            if arm in active_arms and not interval_values and not covered_at_start:
                failures.append(f"active_action_missing:{topic}")
            elif arm in active_arms and not covered_at_start:
                warnings.append(f"active_action_starts_after_episode_start:{topic}")
            elif arm not in active_arms and not covered_at_start:
                warnings.append(f"inactive_action_not_available:{topic}")

    transport_loss = _transport_loss_count(log_file)
    if transport_loss is None:
        warnings.append("transport_loss_stat_unavailable")
    elif transport_loss > 0:
        # rosbag2 reports only a total across all topics. High-rate joint streams can
        # lose an occasional sample without affecting ACT resampling; camera losses
        # are detected precisely above from each raw Image header timestamp.
        warnings.append(f"transport_loss:{transport_loss}")

    return {
        "ok": not failures,
        "bag_dir": str(bag_dir),
        "event_topic": event_topic,
        "start_ns": start_ns or None,
        "end_ns": end_ns or None,
        "duration_sec": round((end_ns - start_ns) / 1e9, 3) if end_ns > start_ns else None,
        "control_hz": control_hz,
        "transport_loss": transport_loss,
        "cameras": camera_report,
        "stream_counts": {topic: len(values) for topic, values in streams.items()},
        "events": events,
        "failures": failures,
        "warnings": warnings,
    }
