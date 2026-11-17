from __future__ import annotations

import os
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class LiveCameraSpec:
    camera_id: str
    name: str
    topic: str
    slot: str


@dataclass(frozen=True)
class LiveCameraFrame:
    seq: int
    stamp_sec: int
    stamp_nanosec: int
    frame_id: str
    recv_time: float
    image_format: str
    data: bytes


class LiveCameraBuffer:
    def __init__(self, spec: LiveCameraSpec, *, stale_after_sec: float = 2.0) -> None:
        self.spec = spec
        self.stale_after_sec = stale_after_sec
        self._condition = threading.Condition()
        self._latest: LiveCameraFrame | None = None
        self._seq = 0
        self._samples: deque[tuple[float, int]] = deque(maxlen=240)
        self._clients = 0
        self._enabled = True
        self._subscribed = False
        self._last_error: str | None = None

    def update_from_ros_msg(self, msg: Any) -> None:
        data = bytes(getattr(msg, "data", b""))
        if not data:
            self.set_error("empty_compressed_image")
            return
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        self.update(
            data=data,
            stamp_sec=int(getattr(stamp, "sec", 0)),
            stamp_nanosec=int(getattr(stamp, "nanosec", 0)),
            frame_id=str(getattr(getattr(msg, "header", None), "frame_id", "")),
            image_format=str(getattr(msg, "format", "jpeg") or "jpeg"),
        )

    def update(
        self,
        *,
        data: bytes,
        stamp_sec: int = 0,
        stamp_nanosec: int = 0,
        frame_id: str = "",
        image_format: str = "jpeg",
    ) -> None:
        now = time.monotonic()
        with self._condition:
            self._seq += 1
            self._latest = LiveCameraFrame(
                seq=self._seq,
                stamp_sec=stamp_sec,
                stamp_nanosec=stamp_nanosec,
                frame_id=frame_id,
                recv_time=now,
                image_format=image_format,
                data=bytes(data),
            )
            stamp_ns = int(stamp_sec) * 1_000_000_000 + int(stamp_nanosec)
            self._samples.append((now, stamp_ns))
            self._last_error = None
            self._condition.notify_all()

    def set_enabled(self, enabled: bool) -> None:
        with self._condition:
            self._enabled = bool(enabled)
            if not self._enabled:
                self._last_error = "camera_disabled"
            self._condition.notify_all()

    def set_subscribed(self, subscribed: bool) -> None:
        with self._condition:
            self._subscribed = bool(subscribed)
            if self._enabled and not self._subscribed:
                self._last_error = "camera_not_subscribed"
            self._condition.notify_all()

    def set_error(self, message: str) -> None:
        with self._condition:
            self._last_error = str(message)
            self._condition.notify_all()

    def enabled(self) -> bool:
        with self._condition:
            return self._enabled

    def subscribed(self) -> bool:
        with self._condition:
            return self._subscribed

    def latest(self) -> LiveCameraFrame | None:
        with self._condition:
            return self._latest

    def wait_for_new_frame(self, last_seq: int, timeout_sec: float = 1.0) -> LiveCameraFrame | None:
        deadline = time.monotonic() + timeout_sec
        with self._condition:
            while self._enabled:
                if self._latest is not None and self._latest.seq != last_seq:
                    return self._latest
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return self._latest
                self._condition.wait(timeout=remaining)
            return None

    def add_client(self) -> None:
        with self._condition:
            self._clients += 1

    def remove_client(self) -> None:
        with self._condition:
            self._clients = max(0, self._clients - 1)

    def status(self) -> dict[str, Any]:
        quality = self.quality_status()
        now = time.monotonic()
        with self._condition:
            age_ms = None
            online = False
            if self._enabled and self._subscribed and self._latest is not None:
                age_sec = now - self._latest.recv_time
                age_ms = int(age_sec * 1000.0)
                online = age_sec <= self.stale_after_sec
            return {
                "id": self.spec.camera_id,
                "name": self.spec.name,
                "topic": self.spec.topic,
                "slot": self.spec.slot,
                "enabled": self._enabled,
                "subscribed": self._subscribed,
                "online": online,
                "fps": quality["fps"],
                "ready": quality["ready"],
                "sample_count": quality["sample_count"],
                "max_gap_ms": quality["max_gap_ms"],
                "timestamps_monotonic": quality["timestamps_monotonic"],
                "last_frame_age_ms": age_ms,
                "seq": self._latest.seq if self._latest else 0,
                "format": self._latest.image_format if self._latest else None,
                "frame_id": self._latest.frame_id if self._latest else None,
                "clients": self._clients,
                "last_error": self._last_error,
                "stream_mjpeg": f"/stream/{self.spec.camera_id}.mjpeg",
            }

    def quality_status(
        self,
        *,
        window_sec: float = 3.0,
        min_fps: float = 29.0,
        max_gap_ms: float = 50.0,
        max_age_ms: float = 100.0,
    ) -> dict[str, Any]:
        now = time.monotonic()
        with self._condition:
            samples = [
                (recv_time, stamp_ns)
                for recv_time, stamp_ns in self._samples
                if now - recv_time <= window_sec
            ]
            fps = 0.0
            if len(samples) >= 2:
                recv_duration = samples[-1][0] - samples[0][0]
                if recv_duration > 0.0:
                    fps = (len(samples) - 1) / recv_duration

            header_stamps = [stamp_ns for _, stamp_ns in samples]
            timestamps_valid = bool(header_stamps) and all(
                stamp_ns > 0 for stamp_ns in header_stamps
            )
            timestamps_monotonic = timestamps_valid and all(
                right > left
                for left, right in zip(header_stamps, header_stamps[1:])
            )
            gaps_ms = [
                (right - left) / 1e6
                for left, right in zip(header_stamps, header_stamps[1:])
            ]
            observed_max_gap_ms = max(gaps_ms) if gaps_ms else None
            age_ms = (
                (now - self._latest.recv_time) * 1000.0
                if self._latest is not None
                else None
            )
            required_samples = max(2, int(math.ceil(window_sec * min_fps)))
            ready = bool(
                self._enabled
                and self._subscribed
                and self._latest is not None
                and age_ms is not None
                and age_ms <= max_age_ms
                and len(samples) >= required_samples
                and fps >= min_fps
                and timestamps_monotonic
                and observed_max_gap_ms is not None
                and observed_max_gap_ms <= max_gap_ms
            )
            return {
                "id": self.spec.camera_id,
                "topic": self.spec.topic,
                "ready": ready,
                "fps": round(fps, 2),
                "sample_count": len(samples),
                "required_samples": required_samples,
                "max_gap_ms": (
                    round(observed_max_gap_ms, 3)
                    if observed_max_gap_ms is not None
                    else None
                ),
                "timestamps_monotonic": timestamps_monotonic,
                "last_frame_age_ms": (
                    round(age_ms, 1) if age_ms is not None else None
                ),
            }

    def availability_status(self, *, max_age_ms: float = 2000.0) -> dict[str, Any]:
        """Report whether this topic is subscribed and has delivered a recent frame."""
        now = time.monotonic()
        with self._condition:
            age_ms = (
                (now - self._latest.recv_time) * 1000.0
                if self._latest is not None
                else None
            )
            ready = bool(
                self._enabled
                and self._subscribed
                and self._latest is not None
                and age_ms is not None
                and age_ms <= max_age_ms
            )
            return {
                "id": self.spec.camera_id,
                "topic": self.spec.topic,
                "ready": ready,
                "subscribed": self._subscribed,
                "received": self._latest is not None,
                "last_frame_age_ms": (
                    round(age_ms, 1) if age_ms is not None else None
                ),
            }


class LiveCameraManager:
    def __init__(self, specs: list[LiveCameraSpec], *, enabled: bool, stale_after_sec: float) -> None:
        self.specs = list(specs)
        self.enabled = enabled
        self.buffers = {
            spec.camera_id: LiveCameraBuffer(spec, stale_after_sec=stale_after_sec)
            for spec in specs
        }
        self._subscriptions: list[Any] = []
        if not enabled:
            for buffer in self.buffers.values():
                buffer.set_enabled(False)

    @classmethod
    def from_environment(cls) -> "LiveCameraManager":
        enabled = _env_bool("DRINK_GRASP_LIVE_CAMERAS_ENABLED", True)
        stale_after_sec = _env_float("DRINK_GRASP_CAMERA_STALE_SEC", 2.0, 0.2, 30.0)
        specs = [
            LiveCameraSpec(
                "cam_high",
                "High Camera",
                _env_topic("DRINK_GRASP_CAM_HIGH_TOPIC", "/cam_high/color/image_raw/compressed"),
                "camera_slot_1",
            ),
            LiveCameraSpec(
                "cam_left",
                "Left Camera",
                _env_topic("DRINK_GRASP_CAM_LEFT_TOPIC", "/cam_left/color/image_raw/compressed"),
                "camera_slot_2",
            ),
            LiveCameraSpec(
                "cam_right",
                "Right Camera",
                _env_topic("DRINK_GRASP_CAM_RIGHT_TOPIC", "/cam_right/color/image_raw/compressed"),
                "camera_slot_3",
            ),
        ]
        return cls(specs, enabled=enabled, stale_after_sec=stale_after_sec)

    def subscribe(self, ros_node: Any, message_type: Any, qos_profile: Any) -> None:
        if not self.enabled:
            return
        for spec in self.specs:
            buffer = self.buffers[spec.camera_id]

            def callback(msg: Any, *, target: LiveCameraBuffer = buffer) -> None:
                target.update_from_ros_msg(msg)

            try:
                subscription = ros_node.create_subscription(
                    message_type,
                    spec.topic,
                    callback,
                    qos_profile,
                )
            except Exception as exc:  # noqa: BLE001 - one camera must not disable Web control.
                buffer.set_subscribed(False)
                buffer.set_error(f"{exc.__class__.__name__}: {exc}")
                continue
            self._subscriptions.append(subscription)
            buffer.set_subscribed(True)

    def set_error(self, message: str) -> None:
        for buffer in self.buffers.values():
            buffer.set_subscribed(False)
            buffer.set_error(message)

    def get(self, camera_id: str) -> LiveCameraBuffer | None:
        return self.buffers.get(camera_id)

    def payload(self) -> dict[str, Any]:
        return {
            "ok": True,
            "source": "drink_grasp_web",
            "integrated": True,
            "cameras": [self.buffers[spec.camera_id].status() for spec in self.specs],
        }

    def readiness_report(
        self,
        *,
        window_sec: float = 3.0,
        min_fps: float = 29.0,
        max_gap_ms: float = 50.0,
        max_age_ms: float = 100.0,
    ) -> dict[str, Any]:
        cameras = [
            self.buffers[spec.camera_id].quality_status(
                window_sec=window_sec,
                min_fps=min_fps,
                max_gap_ms=max_gap_ms,
                max_age_ms=max_age_ms,
            )
            for spec in self.specs
        ]
        return {
            "ready": bool(self.enabled and cameras and all(item["ready"] for item in cameras)),
            "window_sec": window_sec,
            "min_fps": min_fps,
            "max_gap_ms": max_gap_ms,
            "max_age_ms": max_age_ms,
            "cameras": cameras,
        }

    def availability_report(self, *, max_age_ms: float = 2000.0) -> dict[str, Any]:
        cameras = [
            self.buffers[spec.camera_id].availability_status(max_age_ms=max_age_ms)
            for spec in self.specs
        ]
        return {
            "ready": bool(self.enabled and cameras and all(item["ready"] for item in cameras)),
            "mode": "topic_presence",
            "max_age_ms": max_age_ms,
            "cameras": cameras,
        }


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in {"0", "false", "no", "off", "disabled"}


def _env_float(name: str, default: float, minimum: float, maximum: float) -> float:
    try:
        value = float(os.environ.get(name, default))
    except (TypeError, ValueError):
        value = default
    return value if minimum <= value <= maximum else default


def _env_topic(name: str, default: str) -> str:
    topic = str(os.environ.get(name, default)).strip() or default
    return topic if topic.startswith("/") else f"/{topic}"
