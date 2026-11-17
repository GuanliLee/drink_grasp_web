"""Pure validation and sequencing helpers for the dual-arm full workflow.

This module intentionally has no ROS imports.  The Web server owns ROS clients,
while these helpers keep request validation and the left-then-right transition
rules deterministic and unit-testable without moving hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Iterable, Mapping, Sequence


LEFT_ARM = "left"
RIGHT_ARM = "right"
LEFT_CAMERA_FRAME = "cam_left_color_optical_frame"
RIGHT_CAMERA_FRAME = "cam_right_color_optical_frame"
RIGHT_DESTINATION = "default"
RIGHT_PLACE_CONFIG_KEY = "full_flow"
SHELF_POSITION_IDS = tuple(str(index) for index in range(1, 13))

FULL_FLOW_PARAMETER_NAMES = (
    "move_to_capture_pose_before_inference",
    "return_to_initial_pose_after_place",
    "pre_grasp_before_grasp",
    "pre_grasp_offset_m",
    "retreat_after_place",
    "retreat_after_place_offset_m",
    "inference_request_url",
    "full_flow_restock_route_to_place_config",
    "full_flow_pick_route_to_place_config",
    "full_flow_pick_right_capture_joint_enabled",
    "full_flow_pick_right_capture_joint_positions",
)


@dataclass(frozen=True)
class ValidationIssue:
    code: str
    message: str
    field: str = ""

    def as_dict(self) -> dict[str, str]:
        result = {"code": self.code, "message": self.message}
        if self.field:
            result["field"] = self.field
        return result


@dataclass(frozen=True)
class ValidatedPose:
    frame_id: str
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    def as_dict(self) -> dict[str, Any]:
        return {
            "frame_id": self.frame_id,
            "position": list(self.position),
            "orientation_xyzw": list(self.orientation_xyzw),
        }


@dataclass(frozen=True)
class FullFlowStage:
    phase: str
    stage_index: int
    task_mode: str
    service_key: str
    arm: str
    camera_frame: str
    place_label: str
    place_pose: ValidatedPose

    def as_dict(self, *, include_pose: bool = True) -> dict[str, Any]:
        result: dict[str, Any] = {
            "phase": self.phase,
            "stage_index": self.stage_index,
            "stage_count": 2,
            "task_mode": self.task_mode,
            "service_key": self.service_key,
            "arm": self.arm,
            "camera_frame": self.camera_frame,
            "place_label": self.place_label,
        }
        if include_pose:
            result["place_pose"] = self.place_pose.as_dict()
        return result


@dataclass(frozen=True)
class FullFlowInspection:
    drink: Mapping[str, Any] | None
    layer: str
    position: str
    right_destination: str
    scene_id: int
    gripper_width_m: float
    left_pose: ValidatedPose | None
    right_pose: ValidatedPose | None
    issues: tuple[ValidationIssue, ...]

    @property
    def ready(self) -> bool:
        return not self.issues and self.drink is not None and self.left_pose is not None and self.right_pose is not None

    def stages(self) -> tuple[FullFlowStage, FullFlowStage]:
        if not self.ready:
            raise ValueError("full-flow inspection is not ready")
        assert self.left_pose is not None
        assert self.right_pose is not None
        return (
            FullFlowStage(
                phase="LEFT_RESTOCK",
                stage_index=1,
                task_mode="restock",
                service_key="restock",
                arm=LEFT_ARM,
                camera_frame=LEFT_CAMERA_FRAME,
                place_label=f"第{self.layer}层位置{self.position}",
                place_pose=self.left_pose,
            ),
            FullFlowStage(
                phase="RIGHT_PICK",
                stage_index=2,
                task_mode="pick",
                service_key="pick",
                arm=RIGHT_ARM,
                camera_frame=RIGHT_CAMERA_FRAME,
                place_label="右臂最终放置点",
                place_pose=self.right_pose,
            ),
        )

    def request_dict(self) -> dict[str, Any]:
        return {
            "drink_id": str(self.drink.get("id")) if self.drink else "",
            "drink_name_zh": str(self.drink.get("name_zh")) if self.drink else "",
            "prompt": str(self.drink.get("prompt")) if self.drink else "",
            "left_shelf_layer": self.layer,
            "left_shelf_position": self.position,
            "right_destination": self.right_destination,
            "scene_id": self.scene_id,
            "gripper_width_m": self.gripper_width_m,
        }

    def preview_dict(self) -> dict[str, Any]:
        stages: list[dict[str, Any]] = []
        if self.left_pose is not None:
            stages.append(
                FullFlowStage(
                    phase="LEFT_RESTOCK",
                    stage_index=1,
                    task_mode="restock",
                    service_key="restock",
                    arm=LEFT_ARM,
                    camera_frame=LEFT_CAMERA_FRAME,
                    place_label=f"第{self.layer}层位置{self.position}",
                    place_pose=self.left_pose,
                ).as_dict()
            )
        if self.right_pose is not None:
            stages.append(
                FullFlowStage(
                    phase="RIGHT_PICK",
                    stage_index=2,
                    task_mode="pick",
                    service_key="pick",
                    arm=RIGHT_ARM,
                    camera_frame=RIGHT_CAMERA_FRAME,
                    place_label="右臂最终放置点",
                    place_pose=self.right_pose,
                ).as_dict()
            )
        return {
            "ready": self.ready,
            "request": self.request_dict(),
            "stages": stages,
            "issues": [issue.as_dict() for issue in self.issues],
        }


@dataclass(frozen=True)
class SequenceOutcome:
    status: str
    stage_results: tuple[dict[str, Any], ...]
    failed_phase: str = ""

    @property
    def success(self) -> bool:
        return self.status == "completed"


def configured_full_flow_left_slots(
    place_poses: Mapping[str, Any],
) -> tuple[str, ...]:
    configured: list[str] = []
    for layer in ("1", "2", "3"):
        for position in SHELF_POSITION_IDS:
            mapping = _first_mapping(
                place_poses,
                (("arms", LEFT_ARM, "layers", layer, "positions", position),),
            )
            if mapping is None:
                continue
            pose, issues = validate_pose_mapping(
                mapping,
                field="left_shelf_pose",
                label=f"左臂第{layer}层位置{position}",
            )
            if pose is not None and not issues:
                configured.append(f"left:{layer}:{position}")
    return tuple(configured)


def full_flow_right_destination_configured(
    pick_place_poses: Mapping[str, Any],
) -> bool:
    mapping = _first_mapping(
        pick_place_poses,
        (("arms", RIGHT_ARM, RIGHT_PLACE_CONFIG_KEY),),
    )
    if mapping is None:
        return False
    pose, issues = validate_pose_mapping(
        mapping,
        field="right_place_pose",
        label="右臂最终放置位",
    )
    return pose is not None and not issues


def inspect_full_flow(
    body: Mapping[str, Any],
    *,
    drinks: Iterable[Mapping[str, Any]],
    place_poses: Mapping[str, Any],
    pick_place_poses: Mapping[str, Any],
    default_scene_id: int,
    default_gripper_width_m: float,
) -> FullFlowInspection:
    """Validate a full-flow request without applying permissive pose fallbacks."""
    issues: list[ValidationIssue] = []

    drink_id = str(body.get("drink_id") or "").strip()
    drink = next((item for item in drinks if str(item.get("id")) == drink_id), None)
    if drink is None:
        issues.append(
            ValidationIssue("invalid_drink_id", "请选择有效饮料", "drink_id")
        )
    elif not str(drink.get("prompt") or "").strip():
        issues.append(
            ValidationIssue("empty_drink_prompt", "饮料的视觉提示词不能为空", "drink_id")
        )

    layer = str(body.get("left_shelf_layer") or "1").strip()
    if layer not in {"1", "2", "3"}:
        issues.append(
            ValidationIssue("invalid_shelf_layer", "货架层必须是 1、2 或 3", "left_shelf_layer")
        )

    position = str(body.get("left_shelf_position") or "1").strip()
    if position not in SHELF_POSITION_IDS:
        issues.append(
            ValidationIssue(
                "invalid_shelf_position",
                "货架位置必须是 1 到 12",
                "left_shelf_position",
            )
        )

    right_destination = str(body.get("right_destination") or RIGHT_DESTINATION).strip()
    if right_destination != RIGHT_DESTINATION:
        issues.append(
            ValidationIssue(
                "invalid_right_destination",
                "第一版全流程仅支持右臂 default 最终放置点",
                "right_destination",
            )
        )

    scene_id = _validated_int(
        body.get("scene_id"),
        default_scene_id,
        1,
        999,
        "scene_id",
        issues,
    )
    gripper_width_m = _validated_float(
        body.get("gripper_width_m"),
        default_gripper_width_m,
        0.0,
        0.1,
        "gripper_width_m",
        issues,
    )

    left_pose: ValidatedPose | None = None
    if layer in {"1", "2", "3"} and position in SHELF_POSITION_IDS:
        left_mapping = _first_mapping(
            place_poses,
            (
                ("arms", LEFT_ARM, "layers", layer, "positions", position),
            ),
        )
        if left_mapping is None:
            issues.append(
                ValidationIssue(
                    "left_shelf_pose_not_configured",
                    f"左臂第{layer}层位置{position}未配置位姿",
                    "left_shelf_position",
                )
            )
        else:
            left_pose, pose_issues = validate_pose_mapping(
                left_mapping,
                field="left_shelf_pose",
                label=f"左臂第{layer}层位置{position}",
            )
            issues.extend(pose_issues)

    right_mapping = _first_mapping(
        pick_place_poses,
        (
            ("arms", RIGHT_ARM, RIGHT_PLACE_CONFIG_KEY),
        ),
    )
    right_pose: ValidatedPose | None = None
    if right_mapping is None:
        issues.append(
            ValidationIssue(
                "right_place_pose_not_configured",
                "右臂全流程最终放置位未配置：需要 config/pick_place_poses.json 的 arms.right.full_flow",
                "right_destination",
            )
        )
    else:
        right_pose, pose_issues = validate_pose_mapping(
            right_mapping,
            field="right_place_pose",
            label="右臂最终放置位",
        )
        issues.extend(pose_issues)

    return FullFlowInspection(
        drink=drink,
        layer=layer,
        position=position,
        right_destination=right_destination,
        scene_id=scene_id,
        gripper_width_m=gripper_width_m,
        left_pose=left_pose,
        right_pose=right_pose,
        issues=tuple(issues),
    )


def validate_pose_mapping(
    mapping: Mapping[str, Any],
    *,
    field: str,
    label: str,
) -> tuple[ValidatedPose | None, tuple[ValidationIssue, ...]]:
    issues: list[ValidationIssue] = []
    frame_id = str(mapping.get("frame_id") or mapping.get("frame") or "").strip()
    if not frame_id:
        issues.append(
            ValidationIssue("pose_frame_missing", f"{label}缺少 frame_id", field)
        )

    position = mapping.get("position") or mapping.get("xyz")
    orientation = mapping.get("orientation_xyzw") or mapping.get("xyzw")
    position_values = _finite_vector(position, 3)
    orientation_values = _finite_vector(orientation, 4)
    if position_values is None:
        issues.append(
            ValidationIssue("pose_position_invalid", f"{label}位置必须是 3 个有限数", field)
        )
    if orientation_values is None:
        issues.append(
            ValidationIssue(
                "pose_orientation_invalid",
                f"{label}四元数必须是 4 个有限数",
                field,
            )
        )
    elif not 0.95 <= math.sqrt(sum(value * value for value in orientation_values)) <= 1.05:
        issues.append(
            ValidationIssue(
                "pose_quaternion_not_normalized",
                f"{label}四元数模长必须接近 1",
                field,
            )
        )

    if issues or position_values is None or orientation_values is None:
        return None, tuple(issues)
    return (
        ValidatedPose(
            frame_id=frame_id,
            position=(position_values[0], position_values[1], position_values[2]),
            orientation_xyzw=(
                orientation_values[0],
                orientation_values[1],
                orientation_values[2],
                orientation_values[3],
            ),
        ),
        (),
    )


def validate_state_machine_parameters(
    values: Mapping[str, Any],
) -> tuple[ValidationIssue, ...]:
    """Fail closed when active state-machine behavior cannot satisfy the workflow."""
    issues: list[ValidationIssue] = []
    required_true = (
        "move_to_capture_pose_before_inference",
        "return_to_initial_pose_after_place",
        "pre_grasp_before_grasp",
        "retreat_after_place",
    )
    for name in required_true:
        if values.get(name) is not True:
            issues.append(
                ValidationIssue(
                    "state_machine_parameter_mismatch",
                    f"状态机参数 {name} 必须为 true",
                    name,
                )
            )

    pre_grasp_offset = _finite_number(values.get("pre_grasp_offset_m"))
    if pre_grasp_offset is None or pre_grasp_offset <= 0.0:
        issues.append(
            ValidationIssue(
                "state_machine_parameter_mismatch",
                "状态机 pre_grasp_offset_m 必须大于 0",
                "pre_grasp_offset_m",
            )
        )

    retreat_offset = _finite_number(values.get("retreat_after_place_offset_m"))
    if retreat_offset is None or retreat_offset < 0.08 - 1.0e-9:
        issues.append(
            ValidationIssue(
                "state_machine_parameter_mismatch",
                "状态机最小放置后回退距离必须不小于 0.08 m",
                "retreat_after_place_offset_m",
            )
        )

    inference_url = values.get("inference_request_url")
    if not isinstance(inference_url, str) or not inference_url.strip():
        issues.append(
            ValidationIssue(
                "state_machine_parameter_mismatch",
                "状态机 inference_request_url 不能为空",
                "inference_request_url",
            )
        )
    required_route_configs = (
        "full_flow_restock_route_to_place_config",
        "full_flow_pick_route_to_place_config",
    )
    for name in required_route_configs:
        value = values.get(name)
        if not isinstance(value, str) or not value.strip():
            issues.append(
                ValidationIssue(
                    "state_machine_parameter_mismatch",
                    f"状态机途径点配置 {name} 不能为空",
                    name,
                )
            )
    if values.get("full_flow_pick_right_capture_joint_enabled") is not True:
        issues.append(
            ValidationIssue(
                "state_machine_parameter_mismatch",
                "状态机必须启用全流程右臂独立拍照关节位",
                "full_flow_pick_right_capture_joint_enabled",
            )
        )
    full_flow_capture_joints = values.get(
        "full_flow_pick_right_capture_joint_positions"
    )
    if _finite_vector(full_flow_capture_joints, 6) is None:
        issues.append(
            ValidationIssue(
                "state_machine_parameter_mismatch",
                "全流程右臂拍照关节位必须包含 6 个有限弧度值",
                "full_flow_pick_right_capture_joint_positions",
            )
        )
    return tuple(issues)


def run_stage_sequence(
    stages: Sequence[FullFlowStage],
    *,
    run_stage: Callable[[FullFlowStage], dict[str, Any]],
    stop_requested: Callable[[], bool],
    on_phase: Callable[[str, int, FullFlowStage | None], None],
) -> SequenceOutcome:
    """Run stages serially and never enter a later stage after failure or stop."""
    results: list[dict[str, Any]] = []
    for index, stage in enumerate(stages):
        if stop_requested():
            return SequenceOutcome("stopped", tuple(results), stage.phase)
        on_phase(stage.phase, stage.stage_index, stage)
        if stop_requested():
            return SequenceOutcome("stopped", tuple(results), stage.phase)
        result = dict(run_stage(stage))
        result.setdefault("phase", stage.phase)
        result.setdefault("stage_index", stage.stage_index)
        results.append(result)
        if stop_requested() or result.get("status") == "stopped":
            return SequenceOutcome("stopped", tuple(results), stage.phase)
        if not bool(result.get("success")):
            return SequenceOutcome("failed", tuple(results), stage.phase)
        if index + 1 < len(stages):
            on_phase("TRANSITION_CHECK", stage.stage_index, None)
            if stop_requested():
                return SequenceOutcome("stopped", tuple(results), "TRANSITION_CHECK")
    return SequenceOutcome("completed", tuple(results))


def _validated_int(
    raw: Any,
    default: int,
    minimum: int,
    maximum: int,
    field: str,
    issues: list[ValidationIssue],
) -> int:
    if raw is None or raw == "":
        return int(default)
    try:
        value = int(raw)
    except (TypeError, ValueError):
        issues.append(ValidationIssue("invalid_number", f"{field} 必须是整数", field))
        return int(default)
    if not minimum <= value <= maximum:
        issues.append(
            ValidationIssue(
                "number_out_of_range",
                f"{field} 必须在 {minimum} 到 {maximum} 之间",
                field,
            )
        )
        return int(default)
    return value


def _validated_float(
    raw: Any,
    default: float,
    minimum: float,
    maximum: float,
    field: str,
    issues: list[ValidationIssue],
) -> float:
    if raw is None or raw == "":
        return float(default)
    value = _finite_number(raw)
    if value is None:
        issues.append(ValidationIssue("invalid_number", f"{field} 必须是有限数", field))
        return float(default)
    if not minimum <= value <= maximum:
        issues.append(
            ValidationIssue(
                "number_out_of_range",
                f"{field} 必须在 {minimum} 到 {maximum} 之间",
                field,
            )
        )
        return float(default)
    return value


def _finite_number(raw: Any) -> float | None:
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _finite_vector(raw: Any, length: int) -> tuple[float, ...] | None:
    if not isinstance(raw, (list, tuple)) or len(raw) != length:
        return None
    values = tuple(_finite_number(item) for item in raw)
    if any(item is None for item in values):
        return None
    return tuple(float(item) for item in values if item is not None)


def _first_mapping(
    root: Mapping[str, Any],
    paths: Sequence[Sequence[str]],
) -> Mapping[str, Any] | None:
    for path in paths:
        current: Any = root
        for key in path:
            if not isinstance(current, Mapping) or key not in current:
                current = None
                break
            current = current[key]
        if isinstance(current, Mapping):
            return current
    return None
