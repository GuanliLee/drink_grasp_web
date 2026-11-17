from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import live_cameras
from live_cameras import LiveCameraBuffer, LiveCameraManager, LiveCameraSpec


def compressed_message(data: bytes = b"jpeg-data") -> SimpleNamespace:
    return SimpleNamespace(
        data=data,
        format="jpeg",
        header=SimpleNamespace(
            frame_id="camera_frame",
            stamp=SimpleNamespace(sec=12, nanosec=34),
        ),
    )


def test_buffer_reports_online_compressed_frame() -> None:
    buffer = LiveCameraBuffer(
        LiveCameraSpec("cam_test", "Test", "/test/compressed", "slot"),
        stale_after_sec=2.0,
    )
    buffer.set_subscribed(True)
    buffer.update_from_ros_msg(compressed_message())

    status = buffer.status()
    assert status["online"] is True
    assert status["subscribed"] is True
    assert status["seq"] == 1
    assert status["frame_id"] == "camera_frame"
    assert status["stream_mjpeg"] == "/stream/cam_test.mjpeg"
    assert buffer.latest().data == b"jpeg-data"


def test_manager_keeps_three_subscriptions_and_compatible_payload(monkeypatch) -> None:
    for variable in (
        "DRINK_GRASP_LIVE_CAMERAS_ENABLED",
        "DRINK_GRASP_CAM_HIGH_TOPIC",
        "DRINK_GRASP_CAM_LEFT_TOPIC",
        "DRINK_GRASP_CAM_RIGHT_TOPIC",
    ):
        monkeypatch.delenv(variable, raising=False)

    callbacks = {}

    class FakeNode:
        def create_subscription(self, message_type, topic, callback, qos_profile):
            callbacks[topic] = callback
            return (message_type, topic, qos_profile)

    manager = LiveCameraManager.from_environment()
    manager.subscribe(FakeNode(), object(), object())
    callbacks["/cam_right/color/image_raw/compressed"](compressed_message())

    payload = manager.payload()
    assert payload["integrated"] is True
    assert len(payload["cameras"]) == 3
    right = next(camera for camera in payload["cameras"] if camera["id"] == "cam_right")
    assert right["subscribed"] is True
    assert right["online"] is True


def test_buffer_requires_stable_30hz_window_for_recording(monkeypatch) -> None:
    clock = [0.0]
    monkeypatch.setattr(live_cameras.time, "monotonic", lambda: clock[0])
    buffer = LiveCameraBuffer(
        LiveCameraSpec("cam_test", "Test", "/test/compressed", "slot"),
        stale_after_sec=2.0,
    )
    buffer.set_subscribed(True)
    start_ns = 100_000_000_000
    for index in range(90):
        clock[0] = index / 30.0
        stamp_ns = start_ns + round(index * 1e9 / 30.0)
        buffer.update(
            data=b"jpeg",
            stamp_sec=stamp_ns // 1_000_000_000,
            stamp_nanosec=stamp_ns % 1_000_000_000,
        )

    report = buffer.quality_status()
    assert report["ready"] is True
    assert report["sample_count"] == 90
    assert report["fps"] == 30.0
    assert report["max_gap_ms"] < 34.0
    assert report["timestamps_monotonic"] is True


def test_buffer_availability_requires_only_a_recent_received_frame(monkeypatch) -> None:
    clock = [10.0]
    monkeypatch.setattr(live_cameras.time, "monotonic", lambda: clock[0])
    buffer = LiveCameraBuffer(
        LiveCameraSpec("cam_test", "Test", "/test/compressed", "slot"),
        stale_after_sec=2.0,
    )
    buffer.set_subscribed(True)

    assert buffer.availability_status(max_age_ms=2000.0)["ready"] is False
    buffer.update(data=b"jpeg", stamp_sec=12, stamp_nanosec=34)
    assert buffer.availability_status(max_age_ms=2000.0)["ready"] is True
    assert buffer.quality_status()["ready"] is False

    clock[0] += 2.1
    assert buffer.availability_status(max_age_ms=2000.0)["ready"] is False


def test_frontend_defaults_to_current_web_origin() -> None:
    source = (Path(__file__).parents[1] / "static" / "cameras.js").read_text(encoding="utf-8")
    assert "return window.location.origin;" in source
    assert 'host + ":8080"' not in source
