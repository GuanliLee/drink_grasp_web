from __future__ import annotations

import math

from full_flow import (
    inspect_full_flow,
    run_stage_sequence,
    validate_state_machine_parameters,
)


DRINKS = [
    {
        "id": "tea",
        "name_zh": "茶",
        "prompt": "Tea",
    }
]

LEFT_POSES = {
    "arms": {
        "left": {
            "layers": {
                "1": {
                    "positions": {
                        "1": {
                            "frame_id": "world",
                            "position": [0.4, 0.1, 0.5],
                            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                        }
                    }
                }
            }
        }
    }
}

RIGHT_POSES = {
    "arms": {
        "right": {
            "default": {
                "frame_id": "world",
                "position": [0.2, -0.2, 0.3],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "full_flow": {
                "frame_id": "world",
                "position": [0.2, -0.2, 0.33],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            }
        }
    }
}

VALID_PARAMETERS = {
    "move_to_capture_pose_before_inference": True,
    "return_to_initial_pose_after_place": True,
    "pre_grasp_before_grasp": True,
    "pre_grasp_offset_m": 0.08,
    "retreat_after_place": True,
    "retreat_after_place_offset_m": 0.08,
    "inference_request_url": "ws://127.0.0.1:1234",
    "full_flow_restock_route_to_place_config": "left_full_flow_route_to_place.yaml",
    "full_flow_pick_route_to_place_config": "right_full_flow_pick_route_to_place.yaml",
    "full_flow_pick_right_capture_joint_enabled": True,
    "full_flow_pick_right_capture_joint_positions": [
        0.872025560,
        0.909094060,
        -1.470564088,
        -0.300158908,
        0.919176692,
        0.449549324,
    ],
}


def inspect(*, left=LEFT_POSES, right=RIGHT_POSES, body=None):
    return inspect_full_flow(
        body
        or {
            "drink_id": "tea",
            "left_shelf_layer": "1",
            "left_shelf_position": "1",
            "right_destination": "default",
            "scene_id": 1,
            "gripper_width_m": 0.08,
        },
        drinks=DRINKS,
        place_poses=left,
        pick_place_poses=right,
        default_scene_id=1,
        default_gripper_width_m=0.08,
    )


def test_valid_request_builds_fixed_left_then_right_stages():
    result = inspect()
    assert result.ready is True
    stages = result.stages()
    assert [stage.phase for stage in stages] == ["LEFT_RESTOCK", "RIGHT_PICK"]
    assert [stage.arm for stage in stages] == ["left", "right"]
    assert [stage.service_key for stage in stages] == ["restock", "pick"]
    assert stages[1].place_pose.position == (0.2, -0.2, 0.33)


def test_missing_right_destination_fails_closed():
    result = inspect(right={"arms": {"right": {}}})
    assert result.ready is False
    assert "right_place_pose_not_configured" in {issue.code for issue in result.issues}


def test_missing_left_slot_fails_closed():
    result = inspect(left={"arms": {"left": {}}})
    assert result.ready is False
    assert "left_shelf_pose_not_configured" in {issue.code for issue in result.issues}


def test_position_twelve_is_a_valid_but_independently_configured_slot():
    result = inspect(
        body={
            "drink_id": "tea",
            "left_shelf_layer": "1",
            "left_shelf_position": "12",
            "right_destination": "default",
            "scene_id": 3,
            "gripper_width_m": 0.08,
        }
    )
    issue_codes = {issue.code for issue in result.issues}
    assert "invalid_shelf_position" not in issue_codes
    assert "left_shelf_pose_not_configured" in issue_codes


def test_non_finite_pose_is_rejected():
    invalid = {
        "arms": {
            "right": {
                "full_flow": {
                    "frame_id": "world",
                    "position": [0.2, math.nan, 0.3],
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                }
            }
        }
    }
    result = inspect(right=invalid)
    assert result.ready is False
    assert "pose_position_invalid" in {issue.code for issue in result.issues}


def test_active_state_machine_parameters_must_preserve_safety_contract():
    assert validate_state_machine_parameters(VALID_PARAMETERS) == ()
    invalid = {**VALID_PARAMETERS, "retreat_after_place_offset_m": 0.079}
    issues = validate_state_machine_parameters(invalid)
    assert "retreat_after_place_offset_m" in {issue.field for issue in issues}
    missing_route = {**VALID_PARAMETERS, "full_flow_restock_route_to_place_config": ""}
    issues = validate_state_machine_parameters(missing_route)
    assert "full_flow_restock_route_to_place_config" in {
        issue.field for issue in issues
    }


def test_left_failure_never_calls_right_stage():
    stages = inspect().stages()
    calls = []

    def run_stage(stage):
        calls.append(stage.phase)
        return {"success": False, "status": "failed", "message": "left failed"}

    outcome = run_stage_sequence(
        stages,
        run_stage=run_stage,
        stop_requested=lambda: False,
        on_phase=lambda _phase, _index, _stage: None,
    )
    assert outcome.status == "failed"
    assert outcome.failed_phase == "LEFT_RESTOCK"
    assert calls == ["LEFT_RESTOCK"]


def test_transition_stop_never_calls_right_stage():
    stages = inspect().stages()
    calls = []
    stopped = False

    def run_stage(stage):
        calls.append(stage.phase)
        return {"success": True, "status": "completed", "message": "ok"}

    def on_phase(phase, _index, _stage):
        nonlocal stopped
        if phase == "TRANSITION_CHECK":
            stopped = True

    outcome = run_stage_sequence(
        stages,
        run_stage=run_stage,
        stop_requested=lambda: stopped,
        on_phase=on_phase,
    )
    assert outcome.status == "stopped"
    assert calls == ["LEFT_RESTOCK"]


def test_success_calls_both_stages_in_order():
    stages = inspect().stages()
    calls = []
    outcome = run_stage_sequence(
        stages,
        run_stage=lambda stage: calls.append(stage.phase)
        or {"success": True, "status": "completed", "message": "ok"},
        stop_requested=lambda: False,
        on_phase=lambda _phase, _index, _stage: None,
    )
    assert outcome.success is True
    assert calls == ["LEFT_RESTOCK", "RIGHT_PICK"]
