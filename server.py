#!/usr/bin/env python3

from __future__ import annotations

import json
import os
import re
import signal
import shutil
import subprocess
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, replace
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient

from full_flow import (
    FULL_FLOW_PARAMETER_NAMES,
    SHELF_POSITION_IDS,
    FullFlowInspection,
    FullFlowStage,
    ValidationIssue,
    configured_full_flow_left_slots,
    full_flow_right_destination_configured,
    inspect_full_flow,
    run_stage_sequence,
    validate_state_machine_parameters,
)
from playback import EpisodePlaybackLibrary
from live_cameras import LiveCameraManager
from recording_quality import validate_recording

try:
    from graspnet_bridge_interfaces.srv import ExecuteNamedGraspTask
    from std_srvs.srv import Trigger
except ImportError:  # pragma: no cover - depends on sourced ROS overlay
    ExecuteNamedGraspTask = None  # type: ignore[assignment]
    Trigger = None  # type: ignore[assignment]

try:
    from sensor_msgs.msg import CompressedImage, JointState
    from std_msgs.msg import String
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
except ImportError:  # pragma: no cover - depends on sourced ROS environment
    CompressedImage = None  # type: ignore[assignment]
    JointState = None  # type: ignore[assignment]
    String = None  # type: ignore[assignment]
    DurabilityPolicy = None  # type: ignore[assignment]
    QoSProfile = None  # type: ignore[assignment]
    ReliabilityPolicy = None  # type: ignore[assignment]
    qos_profile_sensor_data = None  # type: ignore[assignment]


WEB_DIR = Path(__file__).resolve().parent
STATIC_DIR = WEB_DIR / "static"
PLACE_POSES_FILE = Path(os.environ.get("DRINK_GRASP_PLACE_POSES", WEB_DIR / "config" / "place_poses.json"))
PICK_PLACE_POSES_FILE = Path(
    os.environ.get("DRINK_GRASP_PICK_PLACE_POSES", WEB_DIR / "config" / "pick_place_poses.json")
)
SHELF_PROFILES_FILE = Path(
    os.environ.get(
        "DRINK_GRASP_SHELF_PROFILES",
        WEB_DIR / "config" / "shelf_profiles.json",
    )
)
OMPL_ROUTE_DIR = Path(
    os.environ.get(
        "DRINK_GRASP_OMPL_ROUTE_DIR",
        str(Path.home() / "path_planning_service" / "src" / "path_planning_server" / "config" / "ompl_waypoints"),
    )
)
RECORD_DIR = Path(os.environ.get("DRINK_GRASP_RECORD_DIR", WEB_DIR / "recordings"))

DEFAULT_RECORD_TOPICS = [
    "/cam_high/color/image_raw",
    "/cam_high/color/camera_info",
    "/cam_left/color/image_raw",
    "/cam_left/color/camera_info",
    "/cam_right/color/image_raw",
    "/cam_right/color/camera_info",
    "/joint_left",
    "/joint_right",
    "/joint_states_left",
    "/joint_states_right",
    "/joint_states",
    "/joint_ctrl_cmd_left",
    "/joint_ctrl_cmd_right",
    "/end_pose_stamped_left",
    "/end_pose_stamped_right",
    "/arm_status_left",
    "/arm_status_right",
    "/tf",
    "/tf_static",
    "/act/episode_event",
]

DRINKS = [
    {
        "id": "coca_cola",
        "name_zh": "可口可乐",
        "brand_zh": "Coca-Cola",
        "prompt": "Coca-Cola",
        "tone": "#d92d20",
    },
    {
        "id": "ad_calcium_milk",
        "name_zh": "AD钙奶",
        "brand_zh": "AD Calcium Milk",
        "prompt": "AD Calcium Milk",
        "tone": "#f2b705",
    },
    {
        "id": "daily_c_orange",
        "name_zh": "味全每日C橙汁",
        "brand_zh": "Daily C Orange Juice",
        "prompt": "Daily C Orange Juice",
        "tone": "#f97316",
    },
    {
        "id": "daily_c_grape",
        "name_zh": "味全每日C葡萄汁",
        "brand_zh": "Daily C Grape Juice",
        "prompt": "Daily C Grape Juice",
        "tone": "#7c3aed",
    },
    {
        "id": "wanglaoji",
        "name_zh": "王老吉",
        "brand_zh": "Wanglaoji",
        "prompt": "Wanglaoji",
        "tone": "#b42318",
    },
    {
        "id": "dahongpao_milk_tea",
        "name_zh": "果子熟了大红袍乌龙轻乳茶",
        "brand_zh": "Dahongpao Milk Tea",
        "prompt": "Dahongpao Milk Tea",
        "tone": "#9a3412",
    },
    {
        "id": "sprite",
        "name_zh": "雪碧",
        "brand_zh": "Sprite",
        "prompt": "Sprite",
        "tone": "#16a34a",
    },
    {
        "id": "hk_orange_fanta",
        "name_zh": "港版芬达橙子味",
        "brand_zh": "HK Orange Fanta",
        "prompt": "HK Orange Fanta",
        "tone": "#fb923c",
    },
    {
        "id": "aojiru",
        "name_zh": "伊藤园青汁",
        "brand_zh": "Aojiru",
        "prompt": "Aojiru",
        "tone": "#15803d",
    },
]

ARMS = [
    {"id": "right", "name_zh": "右臂", "camera_frame": "cam_right_color_optical_frame"},
    {"id": "left", "name_zh": "左臂", "camera_frame": "cam_left_color_optical_frame"},
]

SHELF = {
    "layers": [
        {"id": "1", "name_zh": "第一层"},
        {"id": "2", "name_zh": "第二层"},
        {"id": "3", "name_zh": "第三层"},
    ],
    "positions": [
        {"id": position, "name_zh": f"位置 {position}"}
        for position in SHELF_POSITION_IDS
    ],
}


@dataclass
class Runtime:
    host: str
    port: int
    service_name: str
    pick_service_name: str
    full_flow_restock_service_name: str
    full_flow_pick_service_name: str
    cancel_service_name: str
    planning_mode: str
    scene_id: int
    gripper_width_m: float
    service_wait_sec: float
    call_timeout_sec: float
    ros_node: Any | None
    client: Any | None
    pick_client: Any | None
    full_flow_restock_client: Any | None
    full_flow_pick_client: Any | None
    cancel_client: Any | None
    parameter_client: Any | None
    parameter_node_name: str
    ros_error: str | None
    full_flow_stage_timeout_sec: float
    full_flow_total_timeout_sec: float
    place_poses: dict[str, Any]
    pick_place_poses: dict[str, Any]
    shelf_profiles: dict[str, dict[str, Any]]
    default_shelf_profile_id: str
    recorder: Any
    playback: Any
    live_cameras: LiveCameraManager
    lock: threading.Lock
    busy: bool
    current_job: dict[str, Any] | None
    last_result: dict[str, Any] | None


@dataclass
class RecordingSession:
    episode_id: str
    task_name: str
    note: str
    group_name: str
    sequence: int
    output_name: str
    pending_dir: Path
    final_dir: Path
    log_file: Path
    metadata_file: Path
    target_arm: str
    started_at: float
    stopped_at: float | None
    ready_at: float | None
    armed_at: float | None
    episode_started_at: float | None
    episode_ended_at: float | None
    quality_report: dict[str, Any] | None
    status: str
    process: Any | None


class LocalRosbagRecorder:
    def __init__(self, ros_node: Any | None = None, live_cameras: LiveCameraManager | None = None) -> None:
        self.enabled = env_bool("DRINK_GRASP_RECORDING_ENABLED", True)
        self.target_dir = RECORD_DIR
        self.storage = os.environ.get("DRINK_GRASP_RECORD_STORAGE", "mcap").strip() or "mcap"
        self.storage_preset = os.environ.get("DRINK_GRASP_RECORD_STORAGE_PRESET", "zstd_fast").strip()
        self.max_cache_size = bounded_int(
            os.environ.get("DRINK_GRASP_RECORD_MAX_CACHE_SIZE"),
            1024 * 1024 * 1024,
            64 * 1024 * 1024,
            4 * 1024 * 1024 * 1024,
        )
        self.ready_timeout_sec = bounded_float(
            os.environ.get("DRINK_GRASP_RECORD_READY_TIMEOUT_SEC"),
            10.0,
            3.0,
            60.0,
        )
        self.camera_window_sec = bounded_float(
            os.environ.get("DRINK_GRASP_RECORD_CAMERA_WINDOW_SEC"),
            3.0,
            1.0,
            10.0,
        )
        self.camera_min_fps = bounded_float(
            os.environ.get("DRINK_GRASP_RECORD_CAMERA_MIN_FPS"),
            29.0,
            1.0,
            60.0,
        )
        self.camera_max_gap_ms = bounded_float(
            os.environ.get("DRINK_GRASP_RECORD_CAMERA_MAX_GAP_MS"),
            50.0,
            10.0,
            1000.0,
        )
        self.camera_max_delta_ms = bounded_float(
            os.environ.get("DRINK_GRASP_RECORD_CAMERA_MAX_DELTA_MS"),
            25.0,
            5.0,
            100.0,
        )
        self.record_control_hz = bounded_float(
            os.environ.get("DRINK_GRASP_RECORD_CONTROL_HZ"),
            30.0,
            1.0,
            120.0,
        )
        self.topic_max_age_ms = bounded_float(
            os.environ.get(
                "DRINK_GRASP_RECORD_TOPIC_MAX_AGE_MS",
                os.environ.get("DRINK_GRASP_RECORD_CAMERA_MAX_AGE_MS"),
            ),
            2000.0,
            100.0,
            10000.0,
        )
        self.stop_timeout_sec = bounded_float(
            os.environ.get("DRINK_GRASP_RECORD_STOP_TIMEOUT_SEC"),
            30.0,
            1.0,
            180.0,
        )
        self.topics = configured_record_topics()
        self.event_topic = "/act/episode_event"
        self._ros_node = ros_node
        self._live_cameras = live_cameras
        self._lock = threading.RLock()
        self._session: RecordingSession | None = None
        self._last_saved: dict[str, Any] | None = None
        self._last_discarded: dict[str, Any] | None = None
        self._last_error: str | None = None
        self._event_sequence = 0
        self._event_publisher: Any | None = None
        self._subscriptions: list[Any] = []
        self._motion_ready_arms: set[str] = set()
        self._latest_feedback: dict[str, tuple[float, ...] | None] = {
            "left": None,
            "right": None,
        }
        self._feedback_recv_time: dict[str, float | None] = {
            "left": None,
            "right": None,
        }
        self._feedback_history: dict[str, deque[tuple[float, tuple[float, ...]]]] = {
            "left": deque(maxlen=500),
            "right": deque(maxlen=500),
        }
        self._setup_ros_interfaces()

    def _setup_ros_interfaces(self) -> None:
        if (
            self._ros_node is None
            or JointState is None
            or String is None
            or QoSProfile is None
            or ReliabilityPolicy is None
            or DurabilityPolicy is None
        ):
            return
        reliable_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        event_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._event_publisher = self._ros_node.create_publisher(
            String,
            self.event_topic,
            event_qos,
        )
        self._subscriptions.append(
            self._ros_node.create_subscription(
                String,
                self.event_topic,
                self._on_episode_event,
                event_qos,
            )
        )
        for arm in ("left", "right"):
            self._subscriptions.append(
                self._ros_node.create_subscription(
                    JointState,
                    f"/joint_states_{arm}",
                    lambda msg, selected_arm=arm: self._on_feedback(selected_arm, msg),
                    reliable_qos,
                )
            )
            self._subscriptions.append(
                self._ros_node.create_subscription(
                    JointState,
                    f"/joint_ctrl_cmd_{arm}",
                    lambda msg, selected_arm=arm: self._on_command(selected_arm, msg),
                    reliable_qos,
                )
            )

    def _on_feedback(self, arm: str, msg: Any) -> None:
        positions = tuple(float(value) for value in getattr(msg, "position", ()))
        if len(positions) < 6:
            return
        now = time.monotonic()
        with self._lock:
            self._latest_feedback[arm] = positions
            self._feedback_recv_time[arm] = now
            self._feedback_history[arm].append((now, positions))

    def _on_episode_event(self, msg: Any) -> None:
        try:
            payload = json.loads(str(msg.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict) or payload.get("event") != "TASK_MOTION_READY":
            return
        arm = str(payload.get("arm") or "").strip().lower()
        if arm not in {"left", "right"}:
            return
        with self._lock:
            session = self._session
            if (
                session is None
                or session.status != "recording"
                or session.armed_at is None
                or session.episode_started_at is not None
                or not self._session_accepts_arm(session, arm)
            ):
                return
            self._motion_ready_arms.add(arm)

    def _on_command(self, arm: str, msg: Any) -> None:
        command = tuple(float(value) for value in getattr(msg, "position", ()))
        if len(command) < 6:
            return
        with self._lock:
            session = self._session
            if (
                session is None
                or session.status != "recording"
                or session.armed_at is None
                or session.episode_started_at is not None
                or arm not in self._motion_ready_arms
                or not self._session_accepts_arm(session, arm)
            ):
                return
            feedback = self._latest_feedback.get(arm)
            if feedback is None or len(feedback) < 6:
                return
            arm_moving = any(
                abs(command[index] - feedback[index]) >= 0.002
                for index in range(6)
            )
            gripper_moving = bool(
                len(command) >= 7
                and len(feedback) >= 7
                and abs(command[6] - feedback[6]) >= 0.0005
            )
            if not arm_moving and not gripper_moving:
                return
            self._publish_event_locked(
                "START",
                trigger_arm=arm,
                reason="first_motion_command_after_task_ready",
            )

    @staticmethod
    def _session_accepts_arm(session: RecordingSession, arm: str) -> bool:
        if session.target_arm in {"left_right", "right_left", "both", "dual"}:
            return arm in {"left", "right"}
        return session.target_arm == arm

    def _publish_event_locked(
        self,
        event: str,
        *,
        trigger_arm: str | None = None,
        reason: str = "",
    ) -> dict[str, Any] | None:
        session = self._session
        if session is None or self._event_publisher is None or String is None:
            return None
        self._event_sequence += 1
        stamp_ns = (
            int(self._ros_node.get_clock().now().nanoseconds)
            if self._ros_node is not None
            else time.time_ns()
        )
        payload: dict[str, Any] = {
            "event": event,
            "source": "drink_grasp_web",
            "sequence": self._event_sequence,
            "episode_id": session.episode_id,
            "task_name": session.task_name,
            "arm": session.target_arm,
            "trigger_arm": trigger_arm,
            "reason": reason,
            "stamp_ns": stamp_ns,
            "qpos_left": list(self._latest_feedback["left"] or ()),
            "qpos_right": list(self._latest_feedback["right"] or ()),
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._event_publisher.publish(message)
        now = time.time()
        if event == "ARMED":
            session.armed_at = now
        elif event == "START":
            session.episode_started_at = now
        elif event in {"END", "ABORT"}:
            session.episode_ended_at = now
        self._write_metadata_locked(session)
        return payload

    def _robot_readiness_locked(self, *, max_age_ms: float) -> dict[str, Any]:
        now = time.monotonic()
        arms: dict[str, Any] = {}
        for arm in ("left", "right"):
            recv_time = self._feedback_recv_time[arm]
            age_ms = (now - recv_time) * 1000.0 if recv_time is not None else None
            samples = [
                positions
                for sample_time, positions in self._feedback_history[arm]
                if now - sample_time <= 0.5
            ]
            stable = False
            max_arm_delta = None
            max_gripper_delta = None
            if len(samples) >= 10:
                common_size = min(len(item) for item in samples)
                arm_size = min(6, common_size)
                arm_ranges = [
                    max(item[index] for item in samples) - min(item[index] for item in samples)
                    for index in range(arm_size)
                ]
                max_arm_delta = max(arm_ranges, default=0.0)
                if common_size >= 7:
                    gripper_values = [item[6] for item in samples]
                    max_gripper_delta = max(gripper_values) - min(gripper_values)
                else:
                    max_gripper_delta = 0.0
                stable = max_arm_delta <= 0.01 and max_gripper_delta <= 0.0015
            ready = bool(
                self._latest_feedback[arm] is not None
                and age_ms is not None
                and age_ms <= max_age_ms
            )
            arms[arm] = {
                "ready": ready,
                "received": self._latest_feedback[arm] is not None,
                "stable": stable,
                "last_feedback_age_ms": round(age_ms, 1) if age_ms is not None else None,
                "samples_500ms": len(samples),
                "max_arm_delta_rad": (
                    round(max_arm_delta, 6) if max_arm_delta is not None else None
                ),
                "max_gripper_delta": (
                    round(max_gripper_delta, 6) if max_gripper_delta is not None else None
                ),
            }
        return {"ready": all(item["ready"] for item in arms.values()), "arms": arms}

    def _rosbag_subscription_report(self) -> dict[str, Any]:
        missing: list[str] = []
        if self._ros_node is None:
            return {"ready": False, "missing": list(self.topics)}
        for topic in self.topics:
            try:
                endpoints = self._ros_node.get_subscriptions_info_by_topic(topic)
            except Exception:  # noqa: BLE001
                endpoints = []
            if not any(
                str(getattr(endpoint, "node_name", "")).endswith("rosbag2_recorder")
                for endpoint in endpoints
            ):
                missing.append(topic)
        return {"ready": not missing, "missing": missing}

    def _readiness_report_locked(
        self,
        *,
        require_cameras: bool = True,
        require_rosbag: bool = True,
    ) -> dict[str, Any]:
        camera_report = (
            self._live_cameras.availability_report(
                max_age_ms=self.topic_max_age_ms,
            )
            if self._live_cameras is not None
            else {"ready": False, "cameras": []}
        )
        robot_report = self._robot_readiness_locked(max_age_ms=self.topic_max_age_ms)
        rosbag_report = self._rosbag_subscription_report()
        return {
            "ready": bool(
                (not require_cameras or camera_report.get("ready"))
                and robot_report.get("ready")
                and (not require_rosbag or rosbag_report.get("ready"))
            ),
            "required": {
                "mode": "topic_presence",
                "cameras": require_cameras,
                "robot": True,
                "rosbag": require_rosbag,
            },
            "cameras": camera_report,
            "robot": robot_report,
            "rosbag": rosbag_report,
        }

    def status(self) -> dict[str, Any]:
        with self._lock:
            self._refresh_session_locked()
            session = self._session
            readiness = None
            if session is not None and session.status in {"starting", "recording"}:
                readiness = self._readiness_report_locked(
                    require_cameras=session.process is None,
                    require_rosbag=session.process is not None,
                )
            return {
                "enabled": self.enabled,
                "status": self._session.status if self._session else "idle",
                "session": self._session_payload_locked(self._session),
                "target_dir": str(self.target_dir),
                "storage": self.storage,
                "storage_preset": self.storage_preset,
                "max_cache_size": self.max_cache_size,
                "topics": list(self.topics),
                "readiness": readiness,
                "last_saved": self._last_saved,
                "last_discarded": self._last_discarded,
                "last_error": self._last_error,
            }

    def start(self, task_name: str, note: str, target_arm: str = "right") -> dict[str, Any]:
        if not self.enabled:
            return {"ok": False, "error": "recording_disabled", "status": self.status()}
        if not self.topics:
            return {"ok": False, "error": "recording_topics_empty", "status": self.status()}
        with self._lock:
            self._refresh_session_locked()
            if self._session and self._session.status in {
                "starting",
                "recording",
                "stopped",
                "rejected",
                "error",
            }:
                return {
                    "ok": False,
                    "error": "recording_busy",
                    "status": self._session.status,
                    "session": self._session_payload_locked(self._session),
                }
            session = self._allocate_session_locked(task_name, note, target_arm)
            self._session = session
            self._last_error = None
            self._event_sequence = 0
            self._motion_ready_arms.clear()

        # Wait only until camera and joint-state topics have delivered recent data.
        # Rate, gap, and timestamp jitter are diagnostics and must not delay execution.
        preflight_deadline = time.monotonic() + self.ready_timeout_sec
        last_readiness: dict[str, Any] = {}
        while time.monotonic() < preflight_deadline:
            with self._lock:
                if self._session is not session or session.status != "starting":
                    return {
                        "ok": False,
                        "error": "recording_start_cancelled",
                        "recording": self.status(),
                    }
                last_readiness = self._readiness_report_locked(
                    require_cameras=True,
                    require_rosbag=False,
                )
                if last_readiness.get("ready"):
                    break
            time.sleep(0.1)
        else:
            with self._lock:
                session.status = "error"
                session.stopped_at = time.time()
                self._last_error = (
                    "recording preflight timeout: "
                    + json.dumps(last_readiness, ensure_ascii=False, separators=(",", ":"))
                )
                return {
                    "ok": False,
                    "error": "recording_not_ready",
                    "message": self._last_error,
                    "readiness": last_readiness,
                    "recording": self.status(),
                }

        with self._lock:
            command = self._record_command(session)
            try:
                log_handle = session.log_file.open("ab")
                process = subprocess.Popen(
                    command,
                    cwd=str(WEB_DIR),
                    env=os.environ.copy(),
                    stdout=log_handle,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True,
                )
                log_handle.close()
            except Exception as exc:  # noqa: BLE001
                self._last_error = f"{exc.__class__.__name__}: {exc}"
                session.status = "error"
                session.stopped_at = time.time()
                return {
                    "ok": False,
                    "error": "recording_start_failed",
                    "message": self._last_error,
                    "recording": self.status(),
                }
            session.process = process

        deadline = time.monotonic() + self.ready_timeout_sec
        last_readiness = {}
        while time.monotonic() < deadline:
            with self._lock:
                self._refresh_session_locked()
                if self._session is not session or session.status == "error":
                    self._last_error = tail_text(session.log_file)
                    return {
                        "ok": False,
                        "error": "recording_process_exited",
                        "message": self._last_error,
                        "recording": self.status(),
                    }
                self._wait_for_bag_dir_locked(session)
                last_readiness = self._readiness_report_locked(
                    require_cameras=False,
                    require_rosbag=True,
                )
                if last_readiness.get("ready"):
                    session.status = "recording"
                    session.ready_at = time.time()
                    self._publish_event_locked("ARMED", reason="all_required_topics_available")
                    self._write_metadata_locked(session)
                    return {"ok": True, "recording": self.status()}
            time.sleep(0.1)

        with self._lock:
            self._last_error = (
                "recording readiness timeout: "
                + json.dumps(last_readiness, ensure_ascii=False, separators=(",", ":"))
            )
            self._stop_locked()
            session.status = "error"
            self._write_metadata_locked(session)
            return {
                "ok": False,
                "error": "recording_not_ready",
                "message": self._last_error,
                "readiness": last_readiness,
                "recording": self.status(),
            }

    def stop(self) -> dict[str, Any]:
        with self._lock:
            result = self._stop_locked()
            return {**result, "recording": self.status()}

    def finish_execution(self, *, success: bool, message: str = "") -> None:
        with self._lock:
            session = self._session
            if (
                session is None
                or session.status != "recording"
                or session.episode_ended_at is not None
            ):
                return
            if success and session.episode_started_at is not None:
                self._publish_event_locked("END", reason=message or "task_completed")
            else:
                reason = message or (
                    "task_failed" if session.episode_started_at is not None else "no_motion_start"
                )
                self._publish_event_locked("ABORT", reason=reason)

    def save(self, mark: str = "success") -> dict[str, Any]:
        with self._lock:
            self._refresh_session_locked()
            if not self._session:
                return {"ok": False, "error": "no_recording_session", "recording": self.status()}
            if self._session.status in {"starting", "recording"}:
                stop_result = self._stop_locked()
                if not stop_result.get("ok"):
                    return {**stop_result, "recording": self.status()}
            session = self._session
            if session.status not in {"stopped", "rejected", "error"}:
                return {
                    "ok": False,
                    "error": "recording_not_stopped",
                    "status": session.status,
                    "recording": self.status(),
                }
            try:
                quality_report = validate_recording(
                    session.pending_dir,
                    log_file=session.log_file,
                    min_camera_hz=self.camera_min_fps,
                    max_camera_gap_ms=self.camera_max_gap_ms,
                    control_hz=self.record_control_hz,
                    max_camera_delta_ms=self.camera_max_delta_ms,
                )
            except Exception as exc:  # noqa: BLE001
                quality_report = {
                    "ok": False,
                    "failures": [f"validator_error:{exc.__class__.__name__}:{exc}"],
                    "warnings": [],
                }
            session.quality_report = quality_report
            quality_file = session.pending_dir / "quality_report.json"
            if session.pending_dir.exists():
                quality_file.write_text(
                    json.dumps(quality_report, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
            if not quality_report.get("ok"):
                session.status = "rejected"
                self._last_error = "recording quality validation failed"
                self._write_metadata_locked(session)
                return {
                    "ok": False,
                    "error": "recording_quality_failed",
                    "message": self._last_error,
                    "quality": quality_report,
                    "recording": self.status(),
                }
            if session.final_dir.exists():
                self._last_error = f"final recording already exists: {session.final_dir}"
                return {"ok": False, "error": "recording_exists", "message": self._last_error}
            session.final_dir.parent.mkdir(parents=True, exist_ok=True)
            session.pending_dir.rename(session.final_dir)
            moved_log = session.final_dir / "rosbag_record.log"
            with contextlib_suppress():
                shutil.move(str(session.log_file), str(moved_log))
            saved = {
                "episode_id": session.episode_id,
                "path": str(session.final_dir),
                "log_file": str(moved_log),
                "mark": clean_record_text(mark, "success", 40),
                "duration_sec": recording_duration(session),
                "quality": quality_report,
                "saved_at": time.time(),
            }
            self._last_saved = saved
            self._session = None
            return {"ok": True, "saved": saved, "recording": self.status()}

    def discard(self) -> dict[str, Any]:
        with self._lock:
            self._refresh_session_locked()
            if not self._session:
                return {"ok": False, "error": "no_recording_session", "recording": self.status()}
            if self._session.status in {"starting", "recording"}:
                stop_result = self._stop_locked()
                if not stop_result.get("ok"):
                    return {**stop_result, "recording": self.status()}
            session = self._session
            shutil.rmtree(session.pending_dir, ignore_errors=True)
            with contextlib_suppress():
                session.log_file.unlink()
            discarded = {
                "episode_id": session.episode_id,
                "path": str(session.pending_dir),
                "duration_sec": recording_duration(session),
                "discarded_at": time.time(),
            }
            self._last_discarded = discarded
            self._session = None
            return {"ok": True, "discarded": discarded, "recording": self.status()}

    def _stop_locked(self) -> dict[str, Any]:
        self._refresh_session_locked()
        if not self._session:
            return {"ok": False, "error": "no_recording_session"}
        session = self._session
        if session.status in {"stopped", "rejected", "error"}:
            return {"ok": True, "status": session.status, "session": self._session_payload_locked(session)}
        process = session.process
        if process is None:
            session.status = "error"
            session.stopped_at = time.time()
            self._last_error = "recording process is missing"
            return {"ok": False, "error": "recording_process_missing"}
        if session.episode_ended_at is None and session.armed_at is not None:
            self._publish_event_locked(
                "ABORT",
                reason=(
                    "recording_stopped_before_task_finished"
                    if session.episode_started_at is not None
                    else "recording_stopped_without_motion_start"
                ),
            )
            time.sleep(0.1)
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"{exc.__class__.__name__}: {exc}"
            return {"ok": False, "error": "recording_stop_signal_failed", "message": self._last_error}
        try:
            process.wait(timeout=self.stop_timeout_sec)
        except subprocess.TimeoutExpired:
            with contextlib_suppress():
                os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                with contextlib_suppress():
                    os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)
            session.status = "error"
            session.stopped_at = time.time()
            self._last_error = "ros2 bag record did not stop gracefully"
            return {"ok": False, "error": "recording_stop_timeout", "message": self._last_error}
        session.stopped_at = time.time()
        session.status = "stopped" if process.returncode in (0, -signal.SIGINT) else "error"
        if session.status == "error":
            self._last_error = tail_text(session.log_file)
        self._write_metadata_locked(session)
        return {"ok": True, "status": session.status, "session": self._session_payload_locked(session)}

    def _refresh_session_locked(self) -> None:
        session = self._session
        if (
            not session
            or session.status not in {"starting", "recording"}
            or session.process is None
        ):
            return
        returncode = session.process.poll()
        if returncode is None:
            return
        session.stopped_at = time.time()
        session.status = "stopped" if returncode in (0, -signal.SIGINT) else "error"
        if session.status == "error":
            self._last_error = tail_text(session.log_file)
        self._write_metadata_locked(session)

    def _allocate_session_locked(
        self,
        task_name: str,
        note: str,
        target_arm: str,
    ) -> RecordingSession:
        task = sanitize_component(task_name or "drink_grasp")
        normalized_arm = str(target_arm or "right").strip().lower()
        if normalized_arm not in {"left", "right", "left_right"}:
            normalized_arm = "right"
        now = datetime.now()
        group_name = f"{now:%Y%m%d}_{task}"
        group_dir = self.target_dir / group_name
        group_dir.mkdir(parents=True, exist_ok=True)
        sequence = 1
        while True:
            output_name = f"episode_{sequence:03d}"
            final_dir = group_dir / output_name
            pending_dir = group_dir / f".{output_name}.pending"
            if not final_dir.exists() and not pending_dir.exists():
                break
            sequence += 1
        return RecordingSession(
            episode_id=f"{group_name}/{output_name}",
            task_name=task,
            note=clean_record_text(note, "", 500),
            group_name=group_name,
            sequence=sequence,
            output_name=output_name,
            pending_dir=pending_dir,
            final_dir=final_dir,
            log_file=group_dir / f".{output_name}.rosbag_record.log",
            metadata_file=pending_dir / "drink_recording.json",
            target_arm=normalized_arm,
            started_at=time.time(),
            stopped_at=None,
            ready_at=None,
            armed_at=None,
            episode_started_at=None,
            episode_ended_at=None,
            quality_report=None,
            status="starting",
            process=None,
        )

    def _record_command(self, session: RecordingSession) -> list[str]:
        command = ["ros2", "bag", "record", "-s", self.storage]
        if self.storage_preset:
            command.extend(["--storage-preset-profile", self.storage_preset])
        command.extend(["--max-cache-size", str(self.max_cache_size)])
        command.extend(["-o", str(session.pending_dir)])
        command.append("--topics")
        command.extend(self.topics)
        return command

    def _wait_for_bag_dir_locked(self, session: RecordingSession) -> None:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if session.pending_dir.exists():
                return
            if session.process is not None and session.process.poll() is not None:
                return
            time.sleep(0.1)

    def _write_metadata_locked(self, session: RecordingSession) -> None:
        if not session.pending_dir.exists():
            return
        payload = self._session_payload_locked(session)
        payload["topics"] = list(self.topics)
        session.metadata_file.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def _session_payload_locked(self, session: RecordingSession | None) -> dict[str, Any] | None:
        if session is None:
            return None
        return {
            "episode_id": session.episode_id,
            "task_name": session.task_name,
            "note": session.note,
            "group_name": session.group_name,
            "sequence": session.sequence,
            "output_name": session.output_name,
            "pending_path": str(session.pending_dir),
            "final_path": str(session.final_dir),
            "log_file": str(session.log_file),
            "target_arm": session.target_arm,
            "started_at": session.started_at,
            "stopped_at": session.stopped_at,
            "ready_at": session.ready_at,
            "armed_at": session.armed_at,
            "episode_started_at": session.episode_started_at,
            "episode_ended_at": session.episode_ended_at,
            "quality": session.quality_report,
            "elapsed_sec": recording_duration(session),
            "status": session.status,
            "pid": session.process.pid if session.process is not None else None,
        }


class DrinkGraspServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], runtime: Runtime) -> None:
        super().__init__(address, Handler)
        self.runtime = runtime


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "DrinkGraspWeb/0.1"

    def do_OPTIONS(self) -> None:
        self.send_response(HTTPStatus.NO_CONTENT)
        self._cors()
        self.end_headers()

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)
        if path == "/":
            self._send_file(WEB_DIR / "index.html")
            return
        if path.startswith("/static/"):
            self._send_file(STATIC_DIR / path.removeprefix("/static/"))
            return
        if path == "/api/options":
            self._send_json(
                {
                    "ok": True,
                    "task_modes": [
                        {"id": "restock", "name_zh": "上架"},
                        {"id": "pick", "name_zh": "拣货"},
                        {"id": "full_flow", "name_zh": "全流程"},
                    ],
                    "drinks": DRINKS,
                    "arms": ARMS,
                    "shelf": SHELF,
                    "defaults": {
                        "scene_id": self.server.runtime.scene_id,  # type: ignore[attr-defined]
                        "shelf_profile_id": default_shelf_profile_id(self.server.runtime),  # type: ignore[attr-defined]
                        "gripper_width_m": self.server.runtime.gripper_width_m,  # type: ignore[attr-defined]
                        "planning_mode": self.server.runtime.planning_mode,  # type: ignore[attr-defined]
                    },
                    "shelf_profiles": shelf_profile_options(self.server.runtime),  # type: ignore[attr-defined]
                    "place_pose_config": {
                        "path": str(PLACE_POSES_FILE),
                        "configured_slots": configured_slot_keys(self.server.runtime.place_poses),  # type: ignore[attr-defined]
                    },
                    "pick_place_pose_config": {
                        "path": str(PICK_PLACE_POSES_FILE),
                        "configured_arms": configured_pick_place_arms(self.server.runtime.pick_place_poses),  # type: ignore[attr-defined]
                    },
                    "full_flow": full_flow_options_payload(self.server.runtime),  # type: ignore[attr-defined]
                    "recording": {
                        "enabled": self.server.runtime.recorder.enabled,  # type: ignore[attr-defined]
                        "target_dir": str(self.server.runtime.recorder.target_dir),  # type: ignore[attr-defined]
                        "storage": self.server.runtime.recorder.storage,  # type: ignore[attr-defined]
                        "topics": list(self.server.runtime.recorder.topics),  # type: ignore[attr-defined]
                    },
                }
            )
            return
        if path == "/api/status":
            self._send_json(status_payload(self.server.runtime))  # type: ignore[attr-defined]
            return
        if path == "/api/cameras":
            self._send_json(self.server.runtime.live_cameras.payload())  # type: ignore[attr-defined]
            return
        if path == "/api/full-flow/status":
            self._send_json(full_flow_status_payload(self.server.runtime))  # type: ignore[attr-defined]
            return
        if path == "/api/recording/status":
            self._send_json({"ok": True, "recording": self.server.runtime.recorder.status()})  # type: ignore[attr-defined]
            return
        if path == "/api/playback/episodes":
            episodes = self.server.runtime.playback.catalog()  # type: ignore[attr-defined]
            self._send_json({"ok": True, "episodes": episodes, "count": len(episodes)})
            return
        if path == "/api/playback/manifest":
            episode_id = first_query_value(query, "episode")
            try:
                manifest = self.server.runtime.playback.manifest(episode_id)  # type: ignore[attr-defined]
            except (FileNotFoundError, ValueError) as exc:
                self._send_playback_error(exc)
                return
            except Exception as exc:  # noqa: BLE001
                self._send_playback_error(exc, internal=True)
                return
            self._send_json(manifest)
            return
        if path == "/api/playback/frame":
            episode_id = first_query_value(query, "episode")
            camera_id = first_query_value(query, "camera")
            try:
                frame_index = int(first_query_value(query, "index", "-1"))
                frame = self.server.runtime.playback.frame(  # type: ignore[attr-defined]
                    episode_id, camera_id, frame_index
                )
            except (FileNotFoundError, ValueError, KeyError, IndexError) as exc:
                self._send_playback_error(exc)
                return
            except Exception as exc:  # noqa: BLE001
                self._send_playback_error(exc, internal=True)
                return
            self._send_bytes(frame, "image/jpeg", cache_seconds=3600)
            return
        if path.startswith("/stream/") and path.endswith(".mjpeg"):
            camera_id = path.removeprefix("/stream/").removesuffix(".mjpeg")
            self._stream_mjpeg(camera_id, query)
            return
        self._send_json({"ok": False, "error": "not_found", "path": path}, status=HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        body = self._read_json()
        if path == "/api/execute":
            result = start_execute_job(self.server.runtime, body)  # type: ignore[attr-defined]
            self._send_json(result, status=HTTPStatus.OK if result.get("ok") else HTTPStatus.CONFLICT)
            return
        if path == "/api/full-flow/preview":
            result, _inspection = build_full_flow_preview(self.server.runtime, body)  # type: ignore[attr-defined]
            self._send_json(result)
            return
        if path == "/api/full-flow/execute":
            result = start_full_flow_job(self.server.runtime, body)  # type: ignore[attr-defined]
            self._send_json(result, status=HTTPStatus.OK if result.get("ok") else HTTPStatus.CONFLICT)
            return
        if path == "/api/stop":
            result = stop_execute_job(self.server.runtime)  # type: ignore[attr-defined]
            self._send_json(result, status=HTTPStatus.OK if result.get("ok") else HTTPStatus.CONFLICT)
            return
        if path.startswith("/api/recording/"):
            action = path.removeprefix("/api/recording/")
            result = handle_recording_action(self.server.runtime, action, body)  # type: ignore[attr-defined]
            self._send_json(result, status=HTTPStatus.OK if result.get("ok") else HTTPStatus.CONFLICT)
            return
        self._send_json({"ok": False, "error": "not_found", "path": path}, status=HTTPStatus.NOT_FOUND)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            value = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            return {}
        return value if isinstance(value, dict) else {}

    def _send_file(self, path: Path) -> None:
        try:
            resolved = path.resolve()
            if not str(resolved).startswith(str(WEB_DIR.resolve())):
                raise FileNotFoundError
            data = resolved.read_bytes()
        except FileNotFoundError:
            self._send_json({"ok": False, "error": "not_found"}, status=HTTPStatus.NOT_FOUND)
            return
        content_type = "text/html; charset=utf-8"
        if resolved.suffix == ".css":
            content_type = "text/css; charset=utf-8"
        elif resolved.suffix == ".js":
            content_type = "application/javascript; charset=utf-8"
        self.send_response(HTTPStatus.OK)
        self._cors()
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        if resolved.suffix in {".html", ".js", ".css"}:
            self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _stream_mjpeg(self, camera_id: str, query: dict[str, list[str]]) -> None:
        camera = self.server.runtime.live_cameras.get(camera_id)  # type: ignore[attr-defined]
        if camera is None:
            self._send_json(
                {"ok": False, "error": "camera_not_found", "camera_id": camera_id},
                status=HTTPStatus.NOT_FOUND,
            )
            return
        if not camera.enabled():
            self._send_json(
                {"ok": False, "error": "camera_disabled", "camera_id": camera_id},
                status=HTTPStatus.CONFLICT,
            )
            return
        if not camera.subscribed():
            self._send_json(
                {"ok": False, "error": "camera_not_subscribed", "camera_id": camera_id},
                status=HTTPStatus.SERVICE_UNAVAILABLE,
            )
            return

        max_fps = bounded_float(first_query_value(query, "fps", "12"), 12.0, 1.0, 30.0)
        min_interval = 1.0 / max_fps
        boundary = "drink-grasp-camera"
        self.send_response(HTTPStatus.OK)
        self._cors()
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("X-Stream-Max-Fps", f"{max_fps:g}")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        camera.add_client()
        latest = camera.latest()
        last_seq = latest.seq if latest is not None else 0
        last_sent_at = 0.0
        idle_since = time.monotonic()
        try:
            while True:
                frame = camera.wait_for_new_frame(last_seq, timeout_sec=1.0)
                if frame is None or frame.seq == last_seq:
                    if time.monotonic() - idle_since >= 5.0:
                        return
                    continue
                last_seq = frame.seq
                idle_since = time.monotonic()
                now = time.monotonic()
                if last_sent_at > 0.0 and now - last_sent_at < min_interval:
                    continue
                content_type = "image/png" if "png" in frame.image_format.lower() else "image/jpeg"
                header = (
                    f"--{boundary}\r\n"
                    f"Content-Type: {content_type}\r\n"
                    f"X-Camera-Id: {camera_id}\r\n"
                    f"X-Frame-Seq: {frame.seq}\r\n"
                    f"Content-Length: {len(frame.data)}\r\n"
                    "\r\n"
                ).encode("ascii")
                self.wfile.write(header)
                self.wfile.write(frame.data)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                last_sent_at = time.monotonic()
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            return
        finally:
            camera.remove_client()
            self.close_connection = True

    def _send_bytes(
        self,
        data: bytes,
        content_type: str,
        cache_seconds: int = 0,
    ) -> None:
        self.send_response(HTTPStatus.OK)
        self._cors()
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header(
            "Cache-Control",
            f"public, max-age={cache_seconds}" if cache_seconds else "no-store",
        )
        self.end_headers()
        self.wfile.write(data)

    def _send_playback_error(
        self,
        exc: Exception,
        internal: bool = False,
    ) -> None:
        if internal:
            status = HTTPStatus.INTERNAL_SERVER_ERROR
            error = "playback_decode_failed"
        elif isinstance(exc, ValueError):
            status = HTTPStatus.BAD_REQUEST
            error = "invalid_playback_request"
        else:
            status = HTTPStatus.NOT_FOUND
            error = "playback_data_not_found"
        self._send_json(
            {
                "ok": False,
                "error": error,
                "message": f"{exc.__class__.__name__}: {exc}",
            },
            status=status,
        )

    def _send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self._cors()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"{self.log_date_time_string()} {self.address_string()} {fmt % args}")


def first_query_value(
    query: dict[str, list[str]],
    name: str,
    fallback: str = "",
) -> str:
    values = query.get(name) or []
    return str(values[0]) if values else fallback


def full_flow_options_payload(runtime: Runtime) -> dict[str, Any]:
    profiles = shelf_profile_options(runtime)
    selected = next(
        (
            profile
            for profile in profiles
            if profile["id"] == default_shelf_profile_id(runtime)
        ),
        profiles[0] if profiles else {},
    )
    selected_full_flow = selected.get("full_flow") or {}
    return {
        "workflow": "left_restock_then_right_pick",
        "left_arm": "left",
        "right_arm": "right",
        "right_destination": "default",
        "configured_left_slots": list(
            selected_full_flow.get("configured_left_slots") or []
        ),
        "right_destination_configured": bool(
            selected_full_flow.get("right_destination_configured")
        ),
        "shelf_profiles": {
            profile["id"]: profile.get("full_flow") or {}
            for profile in profiles
        },
        "parameter_node": runtime.parameter_node_name,
        "stage_timeout_sec": runtime.full_flow_stage_timeout_sec,
        "total_timeout_sec": runtime.full_flow_total_timeout_sec,
        "minimum_retreat_m": 0.08,
    }


def full_flow_status_payload(runtime: Runtime) -> dict[str, Any]:
    with runtime.lock:
        current = (
            runtime.current_job
            if runtime.current_job
            and runtime.current_job.get("task_mode") == "full_flow"
            else None
        )
        last = (
            runtime.last_result
            if runtime.last_result
            and runtime.last_result.get("task_mode") == "full_flow"
            else None
        )
        return {
            "ok": True,
            "busy": runtime.busy,
            "current_job": current,
            "last_result": last,
            "options": full_flow_options_payload(runtime),
        }


def build_full_flow_preview(
    runtime: Runtime,
    body: dict[str, Any],
) -> tuple[dict[str, Any], FullFlowInspection]:
    profile, profile_issue = resolve_shelf_profile(runtime, body)
    selected_place_poses = (
        profile.get("place_poses", runtime.place_poses)
        if profile is not None
        else runtime.place_poses
    )
    selected_pick_place_poses = (
        profile.get("pick_place_poses", runtime.pick_place_poses)
        if profile is not None
        else runtime.pick_place_poses
    )
    selected_scene_id = (
        int(profile["scene_id"]) if profile is not None else runtime.scene_id
    )
    inspection = inspect_full_flow(
        body,
        drinks=DRINKS,
        place_poses=selected_place_poses,
        pick_place_poses=selected_pick_place_poses,
        default_scene_id=selected_scene_id,
        default_gripper_width_m=runtime.gripper_width_m,
    )
    extra_issues: list[ValidationIssue] = []
    if profile_issue is not None:
        extra_issues.append(profile_issue)
    if profile is not None:
        owner = shelf_slot_owner(profile, inspection.position)
        if owner and owner != "left":
            extra_issues.append(
                ValidationIssue(
                    "shelf_slot_arm_mismatch",
                    f"{profile['name_zh']}位置{inspection.position}固定由右臂使用；全流程左臂上架请选择位置1/2/3",
                    "left_shelf_position",
                )
            )
        for route_key, arm_name, label in (
            ("full_flow_left_restock", "left", "左臂全流程上架"),
            ("full_flow_right_pick", "right", "右臂全流程拣货"),
        ):
            route_status = shelf_profile_slot_route_status(
                profile,
                route_key,
                arm_name,
                inspection.layer,
                inspection.position,
            )
            if not route_status.get("enabled"):
                extra_issues.append(
                    ValidationIssue(
                        "shelf_route_not_ready",
                        f"{profile['name_zh']}第{inspection.layer}层位置{inspection.position}的{label}途经点尚未示教启用",
                        "left_shelf_position",
                    )
                )
    if extra_issues:
        inspection = replace(
            inspection,
            issues=(*inspection.issues, *extra_issues),
        )
    preview = inspection.preview_dict()
    if profile is not None:
        preview["shelf_profile"] = public_shelf_profile(profile)
        preview["request"]["shelf_profile_id"] = profile["id"]
    preview["ok"] = True
    preview["workflow"] = "left_restock_then_right_pick"
    preview["services"] = {
        "left_restock": runtime.full_flow_restock_service_name,
        "right_pick": runtime.full_flow_pick_service_name,
        "cancel": runtime.cancel_service_name,
    }
    preview["timeouts"] = {
        "stage_sec": runtime.full_flow_stage_timeout_sec,
        "total_sec": runtime.full_flow_total_timeout_sec,
    }

    runtime_issues: tuple[ValidationIssue, ...] = ()
    state_machine: dict[str, Any] = {
        "checked": False,
        "parameter_node": runtime.parameter_node_name,
        "parameters": {},
    }
    if not inspection.issues:
        state_machine, runtime_issues = full_flow_runtime_snapshot(runtime)
    all_issues = [*preview["issues"], *(issue.as_dict() for issue in runtime_issues)]
    preview["issues"] = all_issues
    preview["state_machine"] = state_machine
    preview["ready"] = inspection.ready and not runtime_issues
    return preview, inspection


def full_flow_runtime_snapshot(
    runtime: Runtime,
) -> tuple[dict[str, Any], tuple[ValidationIssue, ...]]:
    snapshot: dict[str, Any] = {
        "checked": False,
        "parameter_node": runtime.parameter_node_name,
        "parameters": {},
    }
    issues: list[ValidationIssue] = []
    if runtime.ros_error:
        issues.append(
            ValidationIssue("ros_unavailable", runtime.ros_error, "ros")
        )
        return snapshot, tuple(issues)
    required_clients = (
        (
            runtime.full_flow_restock_client,
            runtime.full_flow_restock_service_name,
            "full_flow_restock_service_unavailable",
        ),
        (
            runtime.full_flow_pick_client,
            runtime.full_flow_pick_service_name,
            "full_flow_pick_service_unavailable",
        ),
        (runtime.cancel_client, runtime.cancel_service_name, "cancel_service_unavailable"),
    )
    for client, service_name, code in required_clients:
        if client is None or not client.wait_for_service(
            timeout_sec=min(runtime.service_wait_sec, 0.5)
        ):
            issues.append(
                ValidationIssue(code, f"ROS 服务不可用: {service_name}", service_name)
            )
    if issues:
        return snapshot, tuple(issues)

    parameter_client = runtime.parameter_client
    if parameter_client is None or not parameter_client.wait_for_services(timeout_sec=1.0):
        issues.append(
            ValidationIssue(
                "parameter_service_unavailable",
                f"状态机参数服务不可用: {runtime.parameter_node_name}",
                "state_machine_parameters",
            )
        )
        return snapshot, tuple(issues)

    future = parameter_client.get_parameters(list(FULL_FLOW_PARAMETER_NAMES))
    done = threading.Event()
    future.add_done_callback(lambda _future: done.set())
    if not done.wait(timeout=2.0):
        with contextlib_suppress():
            future.cancel()
        issues.append(
            ValidationIssue(
                "parameter_query_timeout",
                f"读取状态机参数超时: {runtime.parameter_node_name}",
                "state_machine_parameters",
            )
        )
        return snapshot, tuple(issues)
    try:
        response = future.result()
        parameter_values = list(response.values) if response is not None else []
        if len(parameter_values) != len(FULL_FLOW_PARAMETER_NAMES):
            raise RuntimeError(
                f"expected {len(FULL_FLOW_PARAMETER_NAMES)} values, got {len(parameter_values)}"
            )
        values = {
            name: parameter_value_to_python(value)
            for name, value in zip(FULL_FLOW_PARAMETER_NAMES, parameter_values)
        }
    except Exception as exc:  # noqa: BLE001
        issues.append(
            ValidationIssue(
                "parameter_query_failed",
                f"读取状态机参数失败: {exc}",
                "state_machine_parameters",
            )
        )
        return snapshot, tuple(issues)

    snapshot["checked"] = True
    snapshot["parameters"] = values
    issues.extend(validate_state_machine_parameters(values))
    return snapshot, tuple(issues)


def start_full_flow_job(runtime: Runtime, body: dict[str, Any]) -> dict[str, Any]:
    with runtime.lock:
        if runtime.busy:
            return {
                "ok": False,
                "error": "task_already_running",
                "current_job": runtime.current_job,
            }

    preview, inspection = build_full_flow_preview(runtime, body)
    if not preview.get("ready"):
        issues = list(preview.get("issues") or [])
        return {
            "ok": False,
            "error": "full_flow_not_ready",
            "message": (
                str(issues[0].get("message"))
                if issues
                else "全流程预检未通过"
            ),
            "preview": preview,
        }

    stages = inspection.stages()
    request_data = inspection.request_dict()
    selected_profile, _profile_issue = resolve_shelf_profile(runtime, body)
    with runtime.lock:
        if runtime.busy:
            return {
                "ok": False,
                "error": "task_already_running",
                "current_job": runtime.current_job,
            }
        job_id = uuid.uuid4().hex[:12]
        job = {
            "job_id": job_id,
            "status": "running",
            "phase": "LEFT_RESTOCK",
            "stage_index": 1,
            "stage_count": 2,
            "started_at": time.time(),
            "finished_at": None,
            "task_mode": "full_flow",
            "task_mode_name_zh": "全流程",
            "workflow": "left_restock_then_right_pick",
            **request_data,
            "shelf_profile_id": (
                selected_profile["id"] if selected_profile is not None else "legacy"
            ),
            "shelf_profile_name_zh": (
                selected_profile["name_zh"]
                if selected_profile is not None
                else "兼容配置"
            ),
            "arm": "left",
            "arm_name_zh": "左臂",
            "camera_frame": stages[0].camera_frame,
            "shelf_layer": inspection.layer,
            "shelf_position": inspection.position,
            "service": runtime.full_flow_restock_service_name,
            "services": preview["services"],
            "left_place_pose": stages[0].place_pose.as_dict(),
            "right_place_pose": stages[1].place_pose.as_dict(),
            "state_machine_parameters": preview["state_machine"],
            "stage_results": [],
            "ros_call_active": False,
            "auto_save_recording": body.get("auto_save_recording") is True,
            "recording_phase": None,
        }
        runtime.busy = True
        runtime.current_job = job
        runtime.last_result = None

    thread = threading.Thread(
        target=execute_full_flow_job,
        args=(runtime, job, stages),
        name=f"drink-grasp-full-flow-{job_id}",
        daemon=True,
    )
    thread.start()
    return {"ok": True, "accepted": True, "job": job, "preview": preview}


def finalize_recording_for_job(
    runtime: Runtime,
    job: dict[str, Any],
    result: dict[str, Any],
) -> dict[str, Any] | None:
    recorder = getattr(runtime, "recorder", None)
    if recorder is None:
        return None

    success = bool(result.get("success"))
    message = str(result.get("message") or result.get("error") or "")
    try:
        recorder.finish_execution(success=success, message=message)
    except Exception as exc:  # noqa: BLE001
        if not job.get("auto_save_recording"):
            return None
        return {
            "ok": False,
            "error": "recording_finish_failed",
            "message": f"{exc.__class__.__name__}: {exc}",
            "status": "error",
            "path": None,
            "quality": None,
        }

    if not job.get("auto_save_recording"):
        return None

    # Give the reliable END/ABORT event time to reach rosbag before SIGINT stops it.
    time.sleep(0.1)
    with runtime.lock:
        if runtime.current_job is job:
            job["recording_phase"] = "validating"
    try:
        response = recorder.save(mark="success" if success else "task_failed")
    except Exception as exc:  # noqa: BLE001
        response = {
            "ok": False,
            "error": "recording_finalize_failed",
            "message": f"{exc.__class__.__name__}: {exc}",
        }

    saved = response.get("saved") if isinstance(response.get("saved"), dict) else {}
    recording = (
        response.get("recording")
        if isinstance(response.get("recording"), dict)
        else {}
    )
    summary = {
        "ok": bool(response.get("ok")),
        "error": response.get("error"),
        "message": response.get("message"),
        "status": "saved" if response.get("ok") else recording.get("status", "error"),
        "path": saved.get("path"),
        "quality": response.get("quality") or saved.get("quality"),
    }
    with runtime.lock:
        if runtime.current_job is job:
            job["recording_phase"] = summary["status"]
    return summary


def execute_full_flow_job(
    runtime: Runtime,
    job: dict[str, Any],
    stages: tuple[FullFlowStage, FullFlowStage],
) -> None:
    started = time.monotonic()
    print(
        "[TIMING] web phase=full_flow_start "
        f"job_id={job.get('job_id')} target={job.get('prompt')!r}",
        flush=True,
    )

    def stop_requested() -> bool:
        with runtime.lock:
            return job.get("status") == "stopping"

    def on_phase(
        phase: str,
        stage_index: int,
        stage: FullFlowStage | None,
    ) -> None:
        with runtime.lock:
            if runtime.current_job is not job:
                return
            job["phase"] = phase
            job["stage_index"] = stage_index
            if stage is not None:
                job["arm"] = stage.arm
                job["arm_name_zh"] = "左臂" if stage.arm == "left" else "右臂"
                job["camera_frame"] = stage.camera_frame
                job["service"] = (
                    runtime.full_flow_restock_service_name
                    if stage.service_key == "restock"
                    else runtime.full_flow_pick_service_name
                )

    def run_stage(stage: FullFlowStage) -> dict[str, Any]:
        remaining_total = runtime.full_flow_total_timeout_sec - (
            time.monotonic() - started
        )
        if remaining_total <= 0.0:
            result = {
                "success": False,
                "status": "failed",
                "error": "full_flow_total_timeout",
                "message": "全流程总超时，未启动下一阶段",
                "phase": stage.phase,
            }
        else:
            result = call_full_flow_stage(
                runtime,
                job,
                stage,
                timeout_sec=min(
                    runtime.full_flow_stage_timeout_sec,
                    remaining_total,
                ),
            )
        with runtime.lock:
            if runtime.current_job is job:
                job["stage_results"].append(result)
        return result

    try:
        outcome = run_stage_sequence(
            stages,
            run_stage=run_stage,
            stop_requested=stop_requested,
            on_phase=on_phase,
        )
        if outcome.status == "completed":
            result = {
                "ok": True,
                "success": True,
                "status": "completed",
                "phase": "COMPLETED",
                "message": "全流程完成：左臂上架和右臂拣货均成功",
            }
        elif outcome.status == "stopped":
            result = {
                "ok": False,
                "success": False,
                "status": "stopped",
                "phase": "STOPPED",
                "error": "canceled",
                "failed_phase": outcome.failed_phase,
                "message": "全流程已停止，未启动后续阶段",
            }
        else:
            last_stage = outcome.stage_results[-1] if outcome.stage_results else {}
            result = {
                "ok": False,
                "success": False,
                "status": "failed",
                "phase": "FAILED",
                "error": str(last_stage.get("error") or "state_machine_failed"),
                "failed_phase": outcome.failed_phase,
                "message": str(last_stage.get("message") or "全流程阶段失败"),
            }
    except Exception as exc:  # noqa: BLE001
        with runtime.lock:
            failed_phase = str(job.get("phase") or "UNKNOWN")
        result = {
            "ok": False,
            "success": False,
            "status": "failed",
            "phase": "FAILED",
            "error": exc.__class__.__name__,
            "failed_phase": failed_phase,
            "message": str(exc),
        }

    result["elapsed_ms"] = round((time.monotonic() - started) * 1000.0, 1)
    print(
        "[TIMING] web phase=full_flow_finish "
        f"job_id={job.get('job_id')} success={result.get('success')} "
        f"status={result.get('status')} failed_phase={result.get('failed_phase', '')!r} "
        f"elapsed_ms={result.get('elapsed_ms')}",
        flush=True,
    )
    recording_result = finalize_recording_for_job(runtime, job, result)
    if recording_result is not None:
        result["recording_result"] = recording_result
    with runtime.lock:
        if runtime.current_job is job:
            completed = {
                **job,
                **result,
                "ros_call_active": False,
                "finished_at": time.time(),
            }
            runtime.last_result = completed
            runtime.current_job = None
            runtime.busy = False


def call_full_flow_stage(
    runtime: Runtime,
    job: dict[str, Any],
    stage: FullFlowStage,
    *,
    timeout_sec: float,
) -> dict[str, Any]:
    stage_started = time.monotonic()
    client = (
        runtime.full_flow_restock_client
        if stage.service_key == "restock"
        else runtime.full_flow_pick_client
    )
    service_name = (
        runtime.full_flow_restock_service_name
        if stage.service_key == "restock"
        else runtime.full_flow_pick_service_name
    )
    if runtime.ros_error:
        return full_flow_stage_error(stage, "ros_unavailable", runtime.ros_error, stage_started)
    if ExecuteNamedGraspTask is None or client is None:
        return full_flow_stage_error(
            stage,
            "service_client_unavailable",
            f"ROS 服务客户端不可用: {service_name}",
            stage_started,
        )
    if not client.wait_for_service(timeout_sec=runtime.service_wait_sec):
        return full_flow_stage_error(
            stage,
            "service_unavailable",
            f"ROS 服务不可用: {service_name}",
            stage_started,
        )

    with runtime.lock:
        if job.get("status") == "stopping":
            return full_flow_stage_stopped(stage, stage_started)
        job["ros_call_active"] = True

    request = ExecuteNamedGraspTask.Request()
    request.target_name = str(job["prompt"])
    request.arm_name = stage.arm
    request.camera_frame = stage.camera_frame
    request.scene_id = int(job["scene_id"])
    request.shelf_profile_id = str(job.get("shelf_profile_id") or "")
    request.shelf_layer = int(job.get("shelf_layer") or 0)
    request.shelf_position = int(job.get("shelf_position") or 0)
    request.place_tcp_pose = full_flow_pose_message(stage)
    request.gripper_width_m = float(job["gripper_width_m"])
    try:
        future = client.call_async(request)
    except Exception as exc:  # noqa: BLE001
        with runtime.lock:
            if runtime.current_job is job:
                job["ros_call_active"] = False
        return full_flow_stage_error(
            stage,
            exc.__class__.__name__,
            str(exc),
            stage_started,
        )

    done = threading.Event()

    def mark_done(_future: Any) -> None:
        with runtime.lock:
            if runtime.current_job is job:
                job["ros_call_active"] = False
        done.set()

    future.add_done_callback(mark_done)
    if not done.wait(timeout=max(0.1, timeout_sec)):
        with contextlib_suppress():
            future.cancel()
        with runtime.lock:
            if runtime.current_job is job:
                job["ros_call_active"] = False
        cancel_detail = cancel_full_flow_after_timeout(runtime)
        return full_flow_stage_error(
            stage,
            "stage_timeout",
            f"等待 {service_name} 超时 ({timeout_sec:.1f}s); {cancel_detail}",
            stage_started,
        )
    try:
        response = future.result()
    except Exception as exc:  # noqa: BLE001
        return full_flow_stage_error(
            stage,
            exc.__class__.__name__,
            str(exc),
            stage_started,
        )
    if response is None:
        return full_flow_stage_error(
            stage,
            "empty_service_response",
            f"{service_name} 未返回结果",
            stage_started,
        )

    message = str(response.message)
    with runtime.lock:
        stopping = job.get("status") == "stopping"
    stopped = stopping or "cancel" in message.lower()
    success = bool(response.success) and not stopped
    return {
        "phase": stage.phase,
        "stage_index": stage.stage_index,
        "arm": stage.arm,
        "service": service_name,
        "success": success,
        "status": "stopped" if stopped else ("completed" if success else "failed"),
        "error": "canceled" if stopped else ("" if success else "state_machine_failed"),
        "message": "执行已停止" if stopped else message,
        "elapsed_ms": round((time.monotonic() - stage_started) * 1000.0, 1),
    }


def full_flow_pose_message(stage: FullFlowStage) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = stage.place_pose.frame_id
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
        stage.place_pose.position
    )
    (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ) = stage.place_pose.orientation_xyzw
    return pose


def full_flow_stage_error(
    stage: FullFlowStage,
    error: str,
    message: str,
    started: float,
) -> dict[str, Any]:
    return {
        "phase": stage.phase,
        "stage_index": stage.stage_index,
        "arm": stage.arm,
        "success": False,
        "status": "failed",
        "error": error,
        "message": message,
        "elapsed_ms": round((time.monotonic() - started) * 1000.0, 1),
    }


def full_flow_stage_stopped(
    stage: FullFlowStage,
    started: float,
) -> dict[str, Any]:
    result = full_flow_stage_error(
        stage,
        "canceled",
        "执行已停止",
        started,
    )
    result["status"] = "stopped"
    return result


def cancel_full_flow_after_timeout(runtime: Runtime) -> str:
    client = runtime.cancel_client
    if Trigger is None or client is None:
        return "停止服务客户端不可用"
    if not client.wait_for_service(timeout_sec=min(runtime.service_wait_sec, 1.0)):
        return f"停止服务不可用: {runtime.cancel_service_name}"
    future = client.call_async(Trigger.Request())
    done = threading.Event()
    future.add_done_callback(lambda _future: done.set())
    if not done.wait(timeout=2.0):
        return "停止请求已发送，等待状态机退出"
    try:
        response = future.result()
    except Exception as exc:  # noqa: BLE001
        return f"停止请求失败: {exc}"
    return str(response.message) if response is not None else "停止服务未返回结果"


def start_execute_job(runtime: Runtime, body: dict[str, Any]) -> dict[str, Any]:
    task_mode = clean_choice(body.get("task_mode"), {"restock", "pick"}, "restock")
    drink = drink_by_id(str(body.get("drink_id") or ""))
    if drink is None:
        return {"ok": False, "error": "invalid_drink_id", "drink_id": body.get("drink_id")}
    arm = arm_by_id(str(body.get("arm") or "right"))
    if arm is None:
        return {"ok": False, "error": "invalid_arm", "arm": body.get("arm")}
    layer = clean_choice(body.get("shelf_layer"), {"1", "2", "3"}, "1")
    position = clean_choice(body.get("shelf_position"), set(SHELF_POSITION_IDS), "1")
    profile, profile_issue = resolve_shelf_profile(runtime, body)
    if profile_issue is not None or profile is None:
        return {
            "ok": False,
            "error": profile_issue.code if profile_issue else "invalid_shelf_profile",
            "message": profile_issue.message if profile_issue else "货架配置无效",
        }
    scene_id = int(profile["scene_id"])
    gripper_width = bounded_float(body.get("gripper_width_m"), runtime.gripper_width_m, 0.0, 0.1)
    camera_frame = str(body.get("camera_frame") or arm["camera_frame"]).strip() or arm["camera_frame"]
    if task_mode == "pick":
        place_pose, place_pose_configured = pose_for_pick_place(
            profile["pick_place_poses"], arm["id"]
        )
        service_name = runtime.pick_service_name
    else:
        owner = shelf_slot_owner(profile, position)
        if owner and owner != arm["id"]:
            return {
                "ok": False,
                "error": "shelf_slot_arm_mismatch",
                "message": (
                    f"{profile['name_zh']}位置{position}固定由"
                    f"{'左臂' if owner == 'left' else '右臂'}使用"
                ),
            }
        place_pose, place_pose_configured = pose_for_slot(
            profile["place_poses"], arm["id"], layer, position
        )
        service_name = runtime.service_name
        route_status = shelf_profile_slot_route_status(
            profile,
            "standalone_restock",
            arm["id"],
            layer,
            position,
        )
        route_required = bool(profile.get("slot_routes_required")) or arm["id"] == "right"
        if route_required and not route_status.get("enabled"):
            return {
                "ok": False,
                "error": "shelf_route_not_ready",
                "message": (
                    f"{profile['name_zh']}第{layer}层位置{position}的"
                    f"{'左臂' if arm['id'] == 'left' else '右臂'}上架途经点尚未示教启用"
                ),
            }
        if bool(profile.get("slot_routes_required")):
            return_route_status = shelf_profile_slot_route_status(
                profile,
                "standalone_return",
                arm["id"],
                layer,
                position,
            )
            if not return_route_status.get("enabled"):
                return {
                    "ok": False,
                    "error": "shelf_return_route_not_ready",
                    "message": (
                        f"{profile['name_zh']}第{layer}层位置{position}的"
                        f"{'左臂' if arm['id'] == 'left' else '右臂'}返回途经点尚未示教启用"
                    ),
                }

    with runtime.lock:
        if runtime.busy:
            return {
                "ok": False,
                "error": "task_already_running",
                "current_job": runtime.current_job,
            }
        job_id = uuid.uuid4().hex[:12]
        job = {
            "job_id": job_id,
            "status": "running",
            "started_at": time.time(),
            "finished_at": None,
            "task_mode": task_mode,
            "task_mode_name_zh": "拣货" if task_mode == "pick" else "上架",
            "drink_id": drink["id"],
            "drink_name_zh": drink["name_zh"],
            "prompt": drink["prompt"],
            "arm": arm["id"],
            "arm_name_zh": arm["name_zh"],
            "camera_frame": camera_frame,
            "scene_id": scene_id,
            "shelf_profile_id": profile["id"],
            "shelf_profile_name_zh": profile["name_zh"],
            "gripper_width_m": gripper_width,
            "shelf_layer": layer,
            "shelf_position": position,
            "place_pose_configured": place_pose_configured,
            "service": service_name,
            "auto_save_recording": body.get("auto_save_recording") is True,
            "recording_phase": None,
        }
        runtime.busy = True
        runtime.current_job = job
        runtime.last_result = None

    thread = threading.Thread(
        target=execute_job,
        args=(runtime, job, place_pose),
        name=f"drink-grasp-{job_id}",
        daemon=True,
    )
    thread.start()
    return {"ok": True, "accepted": True, "job": job}


def stop_execute_job(runtime: Runtime) -> dict[str, Any]:
    with runtime.lock:
        if not runtime.busy or runtime.current_job is None:
            return {"ok": False, "error": "no_running_task", "message": "当前没有执行中的任务"}
        job = runtime.current_job
        if job.get("status") == "stopping":
            return {"ok": True, "accepted": True, "message": "停止请求已发送", "job": job}
        job["status"] = "stopping"
        job["stop_requested_at"] = time.time()
        full_flow_without_active_ros_call = (
            job.get("task_mode") == "full_flow"
            and not bool(job.get("ros_call_active"))
        )

    if full_flow_without_active_ros_call:
        return {
            "ok": True,
            "accepted": True,
            "message": "停止请求已记录，不会启动全流程后续阶段",
            "job": job,
        }

    client = runtime.cancel_client
    if runtime.ros_error or Trigger is None or client is None:
        with runtime.lock:
            if runtime.current_job is job and job.get("task_mode") != "full_flow":
                job["status"] = "running"
        message = "ROS 停止服务不可用"
        if job.get("task_mode") == "full_flow":
            message += "；已阻止后续阶段，但当前阶段可能无法立即中止"
        return {
            "ok": False,
            "error": "cancel_service_unavailable",
            "message": message,
            "job": job,
        }
    if not client.wait_for_service(timeout_sec=min(runtime.service_wait_sec, 1.0)):
        with runtime.lock:
            if runtime.current_job is job and job.get("task_mode") != "full_flow":
                job["status"] = "running"
        message = f"停止服务不可用: {runtime.cancel_service_name}"
        if job.get("task_mode") == "full_flow":
            message += "；已阻止后续阶段，但当前阶段可能无法立即中止"
        return {
            "ok": False,
            "error": "cancel_service_unavailable",
            "message": message,
            "job": job,
        }

    future = client.call_async(Trigger.Request())
    done = threading.Event()
    future.add_done_callback(lambda _future: done.set())
    if not done.wait(timeout=2.0):
        return {"ok": True, "accepted": True, "message": "停止请求已发送，等待执行链退出", "job": job}
    try:
        response = future.result()
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": exc.__class__.__name__, "message": str(exc), "job": job}
    return {
        "ok": bool(response and response.success),
        "accepted": bool(response and response.success),
        "message": str(response.message) if response else "停止服务未返回结果",
        "job": job,
    }


def handle_recording_action(runtime: Runtime, action: str, body: dict[str, Any]) -> dict[str, Any]:
    normalized = str(action or "").strip().lower().replace("-", "_")
    if normalized == "start":
        drink = drink_by_id(str(body.get("drink_id") or ""))
        task_mode = clean_choice(
            body.get("task_mode"),
            {"restock", "pick", "full_flow"},
            "restock",
        )
        task_name = clean_record_text(
            body.get("task_name")
            or f"{task_mode}_{drink['id'] if drink else 'drink'}",
            "drink_grasp",
            80,
        )
        note = clean_record_text(body.get("note"), "", 500)
        target_arm = (
            "left_right"
            if task_mode == "full_flow"
            else clean_choice(body.get("arm"), {"left", "right"}, "right")
        )
        return runtime.recorder.start(
            task_name=task_name,
            note=note,
            target_arm=target_arm,
        )
    if normalized == "stop":
        return runtime.recorder.stop()
    if normalized in {"save", "mark", "success"}:
        mark = "success" if normalized in {"mark", "success"} else body.get("mark", "success")
        return runtime.recorder.save(mark=str(mark or "success"))
    if normalized in {"discard", "delete"}:
        return runtime.recorder.discard()
    return {"ok": False, "error": "unknown_recording_action", "action": action}


def execute_job(runtime: Runtime, job: dict[str, Any], place_pose: PoseStamped) -> None:
    result: dict[str, Any]
    started = time.monotonic()
    print(
        "[TIMING] web phase=job_start "
        f"job_id={job.get('id')} task_mode={job.get('task_mode')} "
        f"target={job.get('prompt')!r} arm={job.get('arm')} service={job.get('service')}",
        flush=True,
    )
    try:
        if runtime.ros_error:
            raise RuntimeError(runtime.ros_error)
        if ExecuteNamedGraspTask is None:
            raise RuntimeError("graspnet_bridge_interfaces/srv/ExecuteNamedGraspTask is not available")
        task_mode = str(job.get("task_mode") or "restock")
        client = runtime.pick_client if task_mode == "pick" else runtime.client
        service_name = str(job.get("service") or runtime.service_name)
        if client is None:
            raise RuntimeError("ROS service client is not available")
        if not client.wait_for_service(timeout_sec=runtime.service_wait_sec):
            raise RuntimeError(f"service unavailable: {service_name}")

        request = ExecuteNamedGraspTask.Request()
        request.target_name = str(job["prompt"])
        request.arm_name = str(job["arm"])
        request.camera_frame = str(job["camera_frame"])
        request.scene_id = int(job["scene_id"])
        request.shelf_profile_id = str(job.get("shelf_profile_id") or "")
        request.shelf_layer = int(job.get("shelf_layer") or 0)
        request.shelf_position = int(job.get("shelf_position") or 0)
        request.place_tcp_pose = place_pose
        request.gripper_width_m = float(job["gripper_width_m"])

        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(runtime.call_timeout_sec):
            with contextlib_suppress():
                future.cancel()
            raise TimeoutError(f"timeout waiting for {service_name}: {runtime.call_timeout_sec:.1f}s")
        response = future.result()
        if response is None:
            raise RuntimeError(f"{service_name} returned no response")
        ok = bool(response.success)
        message = str(response.message)
        stopped = job.get("status") == "stopping" or "cancel" in message.lower()
        result = {
            "ok": ok and not stopped,
            "success": ok and not stopped,
            "status": "stopped" if stopped else ("completed" if ok else "failed"),
            "message": "执行已停止" if stopped else message,
            "elapsed_ms": round((time.monotonic() - started) * 1000.0, 1),
        }
        if stopped:
            result["error"] = "canceled"
        elif not ok:
            result["error"] = "state_machine_failed"
    except Exception as exc:  # noqa: BLE001
        stopped = job.get("status") == "stopping"
        result = {
            "ok": False,
            "success": False,
            "status": "stopped" if stopped else "error",
            "error": "canceled" if stopped else exc.__class__.__name__,
            "message": "执行已停止" if stopped else str(exc),
            "elapsed_ms": round((time.monotonic() - started) * 1000.0, 1),
        }

    print(
        "[TIMING] web phase=job_finish "
        f"job_id={job.get('id')} success={bool(result.get('success'))} "
        f"status={result.get('status')} elapsed_ms={result.get('elapsed_ms')} "
        f"message={str(result.get('message'))!r}",
        flush=True,
    )

    recording_result = finalize_recording_for_job(runtime, job, result)
    if recording_result is not None:
        result["recording_result"] = recording_result

    with runtime.lock:
        completed = {
            **job,
            **result,
            "finished_at": time.time(),
        }
        runtime.last_result = completed
        runtime.current_job = None
        runtime.busy = False


def status_payload(runtime: Runtime) -> dict[str, Any]:
    with runtime.lock:
        return {
            "ok": True,
            "busy": runtime.busy,
            "current_job": runtime.current_job,
            "last_result": runtime.last_result,
            "ros": {
                "available": (
                    runtime.ros_error is None
                    and runtime.client is not None
                    and runtime.pick_client is not None
                    and runtime.full_flow_restock_client is not None
                    and runtime.full_flow_pick_client is not None
                    and runtime.cancel_client is not None
                ),
                "error": runtime.ros_error,
                "service": runtime.service_name,
                "pick_service": runtime.pick_service_name,
                "full_flow_restock_service": runtime.full_flow_restock_service_name,
                "full_flow_pick_service": runtime.full_flow_pick_service_name,
                "cancel_service": runtime.cancel_service_name,
                "services": {
                    "restock": runtime.service_name,
                    "pick": runtime.pick_service_name,
                    "full_flow": [
                        runtime.full_flow_restock_service_name,
                        runtime.full_flow_pick_service_name,
                    ],
                },
                "state_machine_parameter_node": runtime.parameter_node_name,
                "domain_id": os.environ.get("ROS_DOMAIN_ID"),
            },
            "planning": {
                "interface": "/plan_to_pose_v2",
                "object_mode": runtime.planning_mode,
                "geometry_source": "121.grasp_goal.mesh_geometry",
                "visual_asset_source": "path_planning_service0807",
            },
            "full_flow": full_flow_options_payload(runtime),
            "recording": runtime.recorder.status(),
        }


def pose_for_slot(
    place_poses: dict[str, Any],
    arm: str,
    layer: str,
    position: str,
) -> tuple[PoseStamped, bool]:
    slot = (
        nested_get(place_poses, ["arms", arm, "layers", layer, "positions", position])
        or nested_get(place_poses, [arm, layer, position])
        or nested_get(place_poses, [layer, position])
    )
    return pose_from_config(slot)


def pose_for_pick_place(place_poses: dict[str, Any], arm: str) -> tuple[PoseStamped, bool]:
    slot = (
        nested_get(place_poses, ["arms", arm, "default"])
        or nested_get(place_poses, ["arms", arm, "place"])
        or nested_get(place_poses, [arm, "default"])
        or nested_get(place_poses, [arm, "place"])
        or nested_get(place_poses, ["default"])
        or nested_get(place_poses, ["place"])
    )
    return pose_from_config(slot)


def pose_from_config(slot: Any) -> tuple[PoseStamped, bool]:
    pose = PoseStamped()
    if not isinstance(slot, dict):
        return pose, False
    frame_id = str(slot.get("frame_id") or slot.get("frame") or "world").strip()
    xyz = slot.get("position") or slot.get("xyz") or [
        slot.get("x"),
        slot.get("y"),
        slot.get("z"),
    ]
    quat = slot.get("orientation_xyzw") or slot.get("xyzw") or [
        slot.get("qx"),
        slot.get("qy"),
        slot.get("qz"),
        slot.get("qw"),
    ]
    try:
        xyz_values = [float(value) for value in xyz]
        quat_values = [float(value) for value in quat]
    except (TypeError, ValueError):
        return pose, False
    if len(xyz_values) != 3 or len(quat_values) != 4:
        return pose, False
    pose.header.frame_id = frame_id or "world"
    pose.pose.position.x = xyz_values[0]
    pose.pose.position.y = xyz_values[1]
    pose.pose.position.z = xyz_values[2]
    pose.pose.orientation.x = quat_values[0]
    pose.pose.orientation.y = quat_values[1]
    pose.pose.orientation.z = quat_values[2]
    pose.pose.orientation.w = quat_values[3]
    return pose, True


def load_place_poses() -> dict[str, Any]:
    if not PLACE_POSES_FILE.exists():
        return {}
    try:
        value = json.loads(PLACE_POSES_FILE.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}
    return value if isinstance(value, dict) else {}


def load_pick_place_poses() -> dict[str, Any]:
    if not PICK_PLACE_POSES_FILE.exists():
        return {}
    try:
        value = json.loads(PICK_PLACE_POSES_FILE.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}
    return value if isinstance(value, dict) else {}


def _load_json_mapping(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def load_shelf_profiles() -> tuple[dict[str, dict[str, Any]], str]:
    root = _load_json_mapping(SHELF_PROFILES_FILE)
    result: dict[str, dict[str, Any]] = {}
    raw_profiles = root.get("profiles")
    if not isinstance(raw_profiles, list):
        return result, ""
    config_dir = SHELF_PROFILES_FILE.parent
    for raw_profile in raw_profiles:
        if not isinstance(raw_profile, dict):
            continue
        profile_id = str(raw_profile.get("id") or "").strip()
        if not profile_id or profile_id in result:
            continue
        try:
            scene_id = int(raw_profile.get("scene_id"))
        except (TypeError, ValueError):
            continue
        if scene_id <= 0:
            continue
        place_name = Path(str(raw_profile.get("place_poses_file") or ""))
        pick_name = Path(str(raw_profile.get("pick_place_poses_file") or ""))
        if (
            not place_name.name
            or place_name.name != str(place_name)
            or not pick_name.name
            or pick_name.name != str(pick_name)
        ):
            continue
        result[profile_id] = {
            **raw_profile,
            "id": profile_id,
            "scene_id": scene_id,
            "name_zh": str(raw_profile.get("name_zh") or profile_id),
            "place_poses": _load_json_mapping(config_dir / place_name),
            "pick_place_poses": _load_json_mapping(config_dir / pick_name),
        }
    default_id = str(root.get("default_profile_id") or "").strip()
    if default_id not in result:
        default_id = next(iter(result), "")
    return result, default_id


def default_shelf_profile_id(runtime: Any) -> str:
    configured = str(getattr(runtime, "default_shelf_profile_id", "") or "")
    profiles = getattr(runtime, "shelf_profiles", None)
    if isinstance(profiles, dict) and configured in profiles:
        return configured
    return "legacy"


def legacy_shelf_profile(runtime: Any) -> dict[str, Any]:
    return {
        "id": "legacy",
        "name_zh": "货架1（兼容配置）",
        "scene_id": int(runtime.scene_id),
        "default_layer": "2",
        "default_position": "3",
        "place_poses": runtime.place_poses,
        "pick_place_poses": runtime.pick_place_poses,
        "routes": {
            "standalone_right_restock": "right_route_to_place.yaml",
            "full_flow_left_restock": "left_full_flow_route_to_place.yaml",
            "full_flow_right_pick": "right_full_flow_pick_route_to_place.yaml",
        },
    }


def runtime_shelf_profiles(runtime: Any) -> dict[str, dict[str, Any]]:
    profiles = getattr(runtime, "shelf_profiles", None)
    if isinstance(profiles, dict) and profiles:
        return profiles
    return {"legacy": legacy_shelf_profile(runtime)}


def resolve_shelf_profile(
    runtime: Any,
    body: dict[str, Any],
) -> tuple[dict[str, Any] | None, ValidationIssue | None]:
    profiles = runtime_shelf_profiles(runtime)
    requested_id = str(body.get("shelf_profile_id") or "").strip()
    requested_scene = body.get("scene_id")
    if not requested_id and requested_scene not in (None, ""):
        try:
            scene_id = int(requested_scene)
        except (TypeError, ValueError):
            scene_id = -1
        requested_id = next(
            (
                profile_id
                for profile_id, profile in profiles.items()
                if int(profile["scene_id"]) == scene_id
            ),
            "",
        )
    if not requested_id:
        requested_id = default_shelf_profile_id(runtime)
    profile = profiles.get(requested_id)
    if profile is None:
        return None, ValidationIssue(
            "invalid_shelf_profile",
            "请选择有效的货架配置",
            "shelf_profile_id",
        )
    if requested_scene not in (None, ""):
        try:
            submitted_scene = int(requested_scene)
        except (TypeError, ValueError):
            submitted_scene = -1
        if submitted_scene != int(profile["scene_id"]):
            return None, ValidationIssue(
                "shelf_scene_mismatch",
                "货架配置与 Scene ID 不一致，已阻止执行",
                "scene_id",
            )
    return profile, None


def route_config_enabled(config_name: str) -> bool:
    name = Path(str(config_name or "").strip())
    if not name.name or name.name != str(name):
        return False
    path = OMPL_ROUTE_DIR / name
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            stripped = line.strip().lower()
            if stripped.startswith("enabled:"):
                return stripped.split(":", 1)[1].strip() == "true"
    except OSError:
        return False
    return False


def shelf_profile_route_status(profile: dict[str, Any]) -> dict[str, dict[str, Any]]:
    routes = profile.get("routes")
    if not isinstance(routes, dict):
        routes = {}
    return {
        key: {
            "config": str(routes.get(key) or ""),
            "enabled": route_config_enabled(str(routes.get(key) or "")),
        }
        for key in (
            "standalone_right_restock",
            "full_flow_left_restock",
            "full_flow_right_pick",
        )
    }


def shelf_slot_owner(profile: dict[str, Any], position: str) -> str:
    owners = profile.get("slot_owners")
    if not isinstance(owners, dict):
        return ""
    owner = str(owners.get(str(position)) or "").strip().lower()
    return owner if owner in {"left", "right"} else ""


def shelf_profile_slot_route_status(
    profile: dict[str, Any],
    route_key: str,
    arm: str,
    layer: str,
    position: str,
) -> dict[str, Any]:
    slot_routes = profile.get("slot_routes")
    route_mapping = slot_routes.get(route_key) if isinstance(slot_routes, dict) else None
    slot_key = f"{arm}:{layer}:{position}"
    config_name = (
        str(route_mapping.get(slot_key) or "")
        if isinstance(route_mapping, dict)
        else ""
    )
    if not config_name and not bool(profile.get("slot_routes_required")):
        legacy_key = {
            "standalone_restock": "standalone_right_restock",
            "standalone_return": "standalone_right_return",
            "full_flow_left_restock": "full_flow_left_restock",
            "full_flow_right_pick": "full_flow_right_pick",
        }.get(route_key, "")
        config_name = str((profile.get("routes") or {}).get(legacy_key) or "")
    return {
        "config": config_name,
        "enabled": route_config_enabled(config_name),
        "slot_key": slot_key,
    }


def shelf_profile_slot_routes_status(profile: dict[str, Any]) -> dict[str, dict[str, Any]]:
    output: dict[str, dict[str, Any]] = {}
    slot_routes = profile.get("slot_routes")
    if not isinstance(slot_routes, dict):
        return output
    for route_key, raw_mapping in slot_routes.items():
        if not isinstance(raw_mapping, dict):
            continue
        output[str(route_key)] = {
            str(slot_key): {
                "config": str(config_name or ""),
                "enabled": route_config_enabled(str(config_name or "")),
            }
            for slot_key, config_name in raw_mapping.items()
        }
    return output


def public_shelf_profile(profile: dict[str, Any]) -> dict[str, Any]:
    return {
        "id": profile["id"],
        "name_zh": profile["name_zh"],
        "scene_id": int(profile["scene_id"]),
        "default_layer": str(profile.get("default_layer") or "1"),
        "default_position": str(profile.get("default_position") or "1"),
        "slot_owners": {
            str(position): shelf_slot_owner(profile, str(position))
            for position in SHELF_POSITION_IDS
            if shelf_slot_owner(profile, str(position))
        },
        "slot_routes_required": bool(profile.get("slot_routes_required")),
    }


def shelf_profile_options(runtime: Any) -> list[dict[str, Any]]:
    options: list[dict[str, Any]] = []
    for profile in runtime_shelf_profiles(runtime).values():
        public = public_shelf_profile(profile)
        public["configured_slots"] = configured_slot_keys(profile["place_poses"])
        public["configured_pick_arms"] = configured_pick_place_arms(
            profile["pick_place_poses"]
        )
        public["routes"] = shelf_profile_route_status(profile)
        public["slot_routes"] = shelf_profile_slot_routes_status(profile)
        public["full_flow"] = {
            "configured_left_slots": list(
                configured_full_flow_left_slots(profile["place_poses"])
            ),
            "right_destination_configured": full_flow_right_destination_configured(
                profile["pick_place_poses"]
            ),
            "routes_ready": all(
                public["routes"][key]["enabled"]
                for key in ("full_flow_left_restock", "full_flow_right_pick")
            ),
        }
        options.append(public)
    return options


def configured_slot_keys(place_poses: dict[str, Any]) -> list[str]:
    keys: list[str] = []
    for arm in ("left", "right"):
        for layer in ("1", "2", "3"):
            for position in SHELF_POSITION_IDS:
                _, configured = pose_for_slot(place_poses, arm, layer, position)
                if configured:
                    keys.append(f"{arm}:{layer}:{position}")
    return keys


def configured_pick_place_arms(place_poses: dict[str, Any]) -> list[str]:
    arms: list[str] = []
    for arm in ("left", "right"):
        _, configured = pose_for_pick_place(place_poses, arm)
        if configured:
            arms.append(arm)
    return arms


def drink_by_id(value: str) -> dict[str, Any] | None:
    return next((drink for drink in DRINKS if drink["id"] == value), None)


def arm_by_id(value: str) -> dict[str, Any] | None:
    return next((arm for arm in ARMS if arm["id"] == value), None)


def clean_choice(value: Any, allowed: set[str], fallback: str) -> str:
    text = str(value or "").strip()
    return text if text in allowed else fallback


def bounded_int(value: Any, default: int, minimum: int, maximum: int) -> int:
    try:
        number = int(value)
    except (TypeError, ValueError):
        number = default
    return max(minimum, min(maximum, number))


def bounded_float(value: Any, default: float, minimum: float, maximum: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = default
    if not minimum <= number <= maximum:
        return default
    return number


def env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in {"0", "false", "no", "off", "disabled"}


def configured_record_topics() -> list[str]:
    configured = os.environ.get("DRINK_GRASP_RECORD_TOPICS", "").strip()
    raw_topics = re.split(r"[\s,]+", configured) if configured else DEFAULT_RECORD_TOPICS
    topics: list[str] = []
    seen: set[str] = set()
    for raw in raw_topics:
        topic = str(raw or "").strip()
        if not topic:
            continue
        if not topic.startswith("/"):
            topic = f"/{topic}"
        if topic in seen:
            continue
        seen.add(topic)
        topics.append(topic)
    return topics


def sanitize_component(value: str) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value or "").strip())
    text = text.strip("._-")
    return text[:80] or "drink_grasp"


def clean_record_text(value: Any, fallback: str, limit: int) -> str:
    text = str(value if value is not None else fallback).strip()
    text = re.sub(r"[\x00-\x1f\x7f]+", " ", text)
    return (text or fallback)[:limit]


def recording_duration(session: RecordingSession) -> float:
    end = session.stopped_at if session.stopped_at is not None else time.time()
    return round(max(0.0, end - session.started_at), 3)


def tail_text(path: Path, max_bytes: int = 4000) -> str:
    try:
        data = path.read_bytes()
    except OSError:
        return ""
    return data[-max_bytes:].decode("utf-8", errors="replace").strip()


def nested_get(mapping: dict[str, Any], keys: list[str]) -> Any:
    current: Any = mapping
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


class contextlib_suppress:
    def __enter__(self) -> None:
        return None

    def __exit__(self, *_args: Any) -> bool:
        return True


def make_runtime(host: str, port: int) -> Runtime:
    ros_node = None
    client = None
    pick_client = None
    full_flow_restock_client = None
    full_flow_pick_client = None
    cancel_client = None
    parameter_client = None
    ros_error = None
    live_cameras = LiveCameraManager.from_environment()
    service_name = os.environ.get("DRINK_GRASP_SERVICE", "/execute_named_grasp_task").strip()
    pick_service_name = os.environ.get("DRINK_GRASP_PICK_SERVICE", "/execute_named_pick_task").strip()
    full_flow_restock_service_name = os.environ.get(
        "DRINK_GRASP_FULL_FLOW_RESTOCK_SERVICE",
        "/execute_named_full_flow_restock_task",
    ).strip()
    full_flow_pick_service_name = os.environ.get(
        "DRINK_GRASP_FULL_FLOW_PICK_SERVICE",
        "/execute_named_full_flow_pick_task",
    ).strip()
    cancel_service_name = os.environ.get(
        "DRINK_GRASP_CANCEL_SERVICE", "/cancel_named_grasp_task"
    ).strip()
    parameter_node_name = os.environ.get(
        "DRINK_GRASP_STATE_MACHINE_NODE", "/grasp_bridge_state_machine"
    ).strip() or "/grasp_bridge_state_machine"
    try:
        if ExecuteNamedGraspTask is None or Trigger is None:
            raise RuntimeError("required ROS service interfaces are unavailable; source the ROS overlays")
        rclpy.init(args=None)
        ros_node = rclpy.create_node("drink_grasp_web")
        client = ros_node.create_client(ExecuteNamedGraspTask, service_name)
        pick_client = ros_node.create_client(ExecuteNamedGraspTask, pick_service_name)
        full_flow_restock_client = ros_node.create_client(
            ExecuteNamedGraspTask,
            full_flow_restock_service_name,
        )
        full_flow_pick_client = ros_node.create_client(
            ExecuteNamedGraspTask,
            full_flow_pick_service_name,
        )
        cancel_client = ros_node.create_client(Trigger, cancel_service_name)
        parameter_client = AsyncParameterClient(ros_node, parameter_node_name)
        if CompressedImage is None or qos_profile_sensor_data is None:
            live_cameras.set_error("sensor_msgs/CompressedImage is unavailable")
        else:
            live_cameras.subscribe(ros_node, CompressedImage, qos_profile_sensor_data)
    except Exception as exc:  # noqa: BLE001
        ros_error = f"{exc.__class__.__name__}: {exc}"
        live_cameras.set_error(f"ROS unavailable: {ros_error}")
    recorder = LocalRosbagRecorder(ros_node=ros_node, live_cameras=live_cameras)
    if ros_node is not None:
        threading.Thread(
            target=rclpy.spin,
            args=(ros_node,),
            name="drink-grasp-ros-spin",
            daemon=True,
        ).start()
    shelf_profiles, configured_default_profile_id = load_shelf_profiles()
    legacy_place_poses = load_place_poses()
    legacy_pick_place_poses = load_pick_place_poses()
    if shelf_profiles and configured_default_profile_id in shelf_profiles:
        default_profile = shelf_profiles[configured_default_profile_id]
        active_place_poses = default_profile["place_poses"]
        active_pick_place_poses = default_profile["pick_place_poses"]
        default_scene_id = int(default_profile["scene_id"])
    else:
        active_place_poses = legacy_place_poses
        active_pick_place_poses = legacy_pick_place_poses
        default_scene_id = bounded_int(
            os.environ.get("DRINK_GRASP_SCENE_ID"), 1, 1, 999
        )
    return Runtime(
        host=host,
        port=port,
        service_name=service_name,
        pick_service_name=pick_service_name,
        full_flow_restock_service_name=full_flow_restock_service_name,
        full_flow_pick_service_name=full_flow_pick_service_name,
        cancel_service_name=cancel_service_name,
        planning_mode=os.environ.get(
            "DRINK_GRASP_OBJECT_MODE",
            "mesh",
        ).strip().lower() or "mesh",
        scene_id=default_scene_id,
        gripper_width_m=bounded_float(os.environ.get("DRINK_GRASP_GRIPPER_WIDTH_M"), 0.08, 0.0, 0.1),
        service_wait_sec=bounded_float(os.environ.get("DRINK_GRASP_SERVICE_WAIT_SEC"), 5.0, 0.0, 60.0),
        call_timeout_sec=bounded_float(os.environ.get("DRINK_GRASP_CALL_TIMEOUT_SEC"), 180.0, 1.0, 900.0),
        ros_node=ros_node,
        client=client,
        pick_client=pick_client,
        full_flow_restock_client=full_flow_restock_client,
        full_flow_pick_client=full_flow_pick_client,
        cancel_client=cancel_client,
        parameter_client=parameter_client,
        parameter_node_name=parameter_node_name,
        ros_error=ros_error,
        full_flow_stage_timeout_sec=bounded_float(
            os.environ.get("DRINK_GRASP_FULL_FLOW_STAGE_TIMEOUT_SEC"),
            300.0,
            1.0,
            900.0,
        ),
        full_flow_total_timeout_sec=bounded_float(
            os.environ.get("DRINK_GRASP_FULL_FLOW_TOTAL_TIMEOUT_SEC"),
            650.0,
            1.0,
            1800.0,
        ),
        place_poses=active_place_poses,
        pick_place_poses=active_pick_place_poses,
        shelf_profiles=shelf_profiles,
        default_shelf_profile_id=configured_default_profile_id,
        recorder=recorder,
        playback=EpisodePlaybackLibrary(RECORD_DIR),
        live_cameras=live_cameras,
        lock=threading.Lock(),
        busy=False,
        current_job=None,
        last_result=None,
    )


def main() -> int:
    host = os.environ.get("DRINK_GRASP_HOST", "0.0.0.0").strip() or "0.0.0.0"
    port = bounded_int(os.environ.get("DRINK_GRASP_PORT"), 8090, 1, 65535)
    runtime = make_runtime(host, port)
    server = DrinkGraspServer((host, port), runtime)

    def shutdown(signum: int, _frame: Any) -> None:
        print(f"received signal {signum}, shutting down")
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f"Drink grasp web listening on http://{host}:{port}/")
    print(
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')} "
        f"service={runtime.service_name} pick_service={runtime.pick_service_name}"
    )
    if runtime.ros_error:
        print(f"ROS unavailable: {runtime.ros_error}")
    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        server.server_close()
        if runtime.ros_node is not None:
            runtime.ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
