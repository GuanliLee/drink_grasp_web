from __future__ import annotations

import struct

from recording_quality import (
    _active_arms,
    _camera_timing_metrics,
    _cdr_header_stamp_ns,
    _transport_loss_count,
)


def test_cdr_header_stamp_parser_uses_ros_header_time() -> None:
    serialized = b"\x00\x01\x00\x00" + struct.pack("<iI", 12, 34)
    assert _cdr_header_stamp_ns(serialized, 99) == 12_000_000_034


def test_cdr_header_stamp_parser_rejects_invalid_stamp() -> None:
    serialized = b"\x00\x01\x00\x00" + struct.pack("<iI", 0, 0)
    assert _cdr_header_stamp_ns(serialized, 99) == 0


def test_dual_arm_event_requires_both_actions() -> None:
    assert _active_arms({"arm": "left_right", "trigger_arm": "left"}) == (
        "left",
        "right",
    )
    assert _active_arms({"arm": "right", "trigger_arm": "right"}) == ("right",)


def test_transport_loss_count_reads_rosbag_summary(tmp_path) -> None:
    log_file = tmp_path / "rosbag_record.log"
    log_file.write_text(
        "[WARN] Number of messages lost on the transport layer: 4\n",
        encoding="utf-8",
    )
    assert _transport_loss_count(log_file) == 4


def test_camera_sync_uses_captured_frames_around_episode_boundaries() -> None:
    start_ns = 1_000_000_000
    end_ns = start_ns + 100_000_000
    metrics = _camera_timing_metrics(
        [
            start_ns - 17_000_000,
            start_ns + 22_000_000,
            start_ns + 55_000_000,
            start_ns + 88_000_000,
            end_ns + 21_000_000,
        ],
        start_ns=start_ns,
        end_ns=end_ns,
        target_times=[start_ns, end_ns],
    )

    assert metrics["frames"] == 3
    assert metrics["max_delta_ms"] == 17.0
