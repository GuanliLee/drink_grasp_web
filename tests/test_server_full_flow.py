from __future__ import annotations

import json
import math
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import server
from rclpy.parameter import Parameter


LEFT_POSE = {
    "frame_id": "world",
    "position": [0.4, 0.1, 0.5],
    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
}
RIGHT_POSE = {
    "frame_id": "world",
    "position": [0.2, -0.2, 0.3],
    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
}
FULL_FLOW_RIGHT_POSE = {
    "frame_id": "world",
    "position": [0.2, -0.2, 0.33],
    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
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


class ReadyClient:
    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return True


class UnavailableClient:
    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return False


class ImmediateFuture:
    def __init__(self, result):
        self._result = result

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._result


class PendingFuture:
    def __init__(self):
        self.cancel_count = 0

    def add_done_callback(self, callback):
        del callback

    def cancel(self):
        self.cancel_count += 1
        return True


class RecordingServiceClient(ReadyClient):
    def __init__(self, *, success=True, message="ok", pending=False):
        self.calls = []
        self.success = success
        self.message = message
        self.pending = pending
        self.pending_future = None

    def call_async(self, request):
        self.calls.append(request)
        if self.pending:
            self.pending_future = PendingFuture()
            return self.pending_future
        return ImmediateFuture(
            SimpleNamespace(success=self.success, message=self.message)
        )


class RecordingCancelClient(ReadyClient):
    def __init__(self):
        self.call_count = 0

    def call_async(self, request):
        del request
        self.call_count += 1
        return ImmediateFuture(SimpleNamespace(success=True, message="canceled"))


class AutoSaveRecorder:
    def __init__(self, save_result=None):
        self.finish_calls = []
        self.save_calls = []
        self.save_result = save_result or {
            "ok": True,
            "saved": {"path": "/tmp/episode_001", "quality": {"ok": True}},
            "recording": {"status": "idle"},
        }

    def finish_execution(self, *, success, message):
        self.finish_calls.append((success, message))

    def save(self, mark):
        self.save_calls.append(mark)
        return self.save_result


class ReadyParameterClient:
    def wait_for_services(self, timeout_sec):
        del timeout_sec
        return True

    def get_parameters(self, names):
        values = [
            Parameter(name, value=VALID_PARAMETERS[name]).get_parameter_value()
            for name in names
        ]
        return ImmediateFuture(SimpleNamespace(values=values))


def runtime(*, full_flow_pose=FULL_FLOW_RIGHT_POSE):
    return SimpleNamespace(
        lock=threading.Lock(),
        busy=False,
        current_job=None,
        last_result=None,
        scene_id=1,
        gripper_width_m=0.08,
        service_wait_sec=0.01,
        call_timeout_sec=1.0,
        service_name="/execute_named_grasp_task",
        pick_service_name="/execute_named_pick_task",
        full_flow_restock_service_name="/execute_named_full_flow_restock_task",
        full_flow_pick_service_name="/execute_named_full_flow_pick_task",
        cancel_service_name="/cancel_named_grasp_task",
        parameter_node_name="/grasp_bridge_state_machine",
        full_flow_stage_timeout_sec=300.0,
        full_flow_total_timeout_sec=650.0,
        ros_error=None,
        client=object(),
        pick_client=object(),
        full_flow_restock_client=object(),
        full_flow_pick_client=object(),
        cancel_client=object(),
        parameter_client=None,
        place_poses={
            "arms": {
                "left": {
                    "layers": {"1": {"positions": {"1": LEFT_POSE}}}
                },
                "right": {
                    "layers": {"1": {"positions": {"1": LEFT_POSE}}}
                },
            }
        },
        pick_place_poses={
            "arms": {
                "right": {
                    "default": RIGHT_POSE,
                    **(
                        {"full_flow": full_flow_pose}
                        if full_flow_pose is not None
                        else {}
                    ),
                },
                "left": {},
            }
        },
    )


def profiled_runtime():
    test_runtime = runtime()
    test_runtime.default_shelf_profile_id = "shelf_1"
    test_runtime.shelf_profiles = {
        "shelf_1": {
            "id": "shelf_1",
            "name_zh": "货架1（原货架）",
            "scene_id": 1,
            "default_layer": "2",
            "default_position": "3",
            "place_poses": test_runtime.place_poses,
            "pick_place_poses": test_runtime.pick_place_poses,
            "routes": {
                "standalone_right_restock": "right_route_to_place.yaml",
                "full_flow_left_restock": "left_full_flow_route_to_place.yaml",
                "full_flow_right_pick": "right_full_flow_pick_route_to_place.yaml",
            },
        },
        "shelf_2": {
            "id": "shelf_2",
            "name_zh": "货架2（新大货架）",
            "scene_id": 3,
            "default_layer": "1",
            "default_position": "5",
            "place_poses": {
                "arms": {
                    "right": {
                        "layers": {"1": {"positions": {"5": RIGHT_POSE}}}
                    },
                    "left": {"layers": {"1": {"positions": {}}}},
                }
            },
            "pick_place_poses": test_runtime.pick_place_poses,
            "routes": {
                "standalone_right_restock": "right_route_to_place_scene_003.yaml",
                "full_flow_left_restock": "left_full_flow_route_to_place_scene_003.yaml",
                "full_flow_right_pick": "right_full_flow_pick_route_to_place_scene_003.yaml",
            },
            "slot_owners": {
                "1": "left", "2": "left", "3": "left",
                "4": "right", "5": "right", "6": "right",
                "7": "left", "8": "left", "9": "left",
                "10": "right", "11": "right", "12": "right",
            },
            "slot_routes_required": True,
            "slot_routes": {
                "standalone_restock": {
                    "right:1:5": "right_route_to_place_scene_003_layer_1_position_5.yaml",
                },
                "full_flow_left_restock": {},
                "full_flow_right_pick": {},
            },
        },
    }
    return test_runtime


def test_actual_large_shelf_outer_row_routes_and_common_x() -> None:
    config_dir = Path(server.WEB_DIR) / "config"
    profiles = json.loads((config_dir / "shelf_profiles.json").read_text())
    shelf_2 = next(profile for profile in profiles["profiles"] if profile["id"] == "shelf_2")
    poses = json.loads((config_dir / "place_poses_shelf_2.json").read_text())
    right_positions = poses["arms"]["right"]["layers"]["1"]["positions"]

    assert shelf_2["slot_owners"] | {
        "7": "left", "8": "left", "9": "left",
        "10": "right", "11": "right", "12": "right",
    } == shelf_2["slot_owners"]
    outer_row_x_m = 0.4552727780508177
    for inner, outer in (("4", "10"), ("5", "11"), ("6", "12")):
        inner_pose = right_positions[inner]
        outer_pose = right_positions[outer]
        assert math.isclose(
            outer_pose["position"][0],
            outer_row_x_m,
            abs_tol=1.0e-12,
        )
        assert outer_pose["position"][0] < inner_pose["position"][0]
        assert outer_pose["position"][1:] == inner_pose["position"][1:]
        assert outer_pose["orientation_xyzw"] == inner_pose["orientation_xyzw"]
        assert shelf_2["slot_routes"]["standalone_restock"][f"right:1:{outer}"] == (
            shelf_2["slot_routes"]["standalone_restock"][f"right:1:{inner}"]
        )

    return_routes = shelf_2["slot_routes"]["standalone_return"]
    assert return_routes["right:1:10"] != return_routes["right:1:4"]
    assert return_routes["right:1:11"] == return_routes["right:1:5"]
    assert return_routes["right:1:12"] == return_routes["right:1:6"]


def test_shelf_profile_rejects_scene_id_mismatch_before_starting_thread():
    test_runtime = profiled_runtime()
    with patch.object(server.threading, "Thread") as thread_class:
        result = server.start_execute_job(
            test_runtime,
            {
                "task_mode": "pick",
                "drink_id": "dahongpao_milk_tea",
                "arm": "right",
                "shelf_profile_id": "shelf_1",
                "scene_id": 3,
            },
        )
    assert result["ok"] is False
    assert result["error"] == "shelf_scene_mismatch"
    thread_class.assert_not_called()


def test_large_shelf_restock_is_blocked_until_its_return_route_is_enabled():
    test_runtime = profiled_runtime()
    with patch.object(server.threading, "Thread") as thread_class:
        result = server.start_execute_job(
            test_runtime,
            {
                "task_mode": "restock",
                "drink_id": "dahongpao_milk_tea",
                "arm": "right",
                "shelf_profile_id": "shelf_2",
                "scene_id": 3,
                "shelf_layer": "1",
                "shelf_position": "5",
            },
        )
    assert result["ok"] is False
    assert result["error"] == "shelf_return_route_not_ready"
    thread_class.assert_not_called()


def test_large_shelf_rejects_right_arm_for_left_owned_position():
    test_runtime = profiled_runtime()
    with patch.object(server.threading, "Thread") as thread_class:
        result = server.start_execute_job(
            test_runtime,
            {
                "task_mode": "restock",
                "drink_id": "dahongpao_milk_tea",
                "arm": "right",
                "shelf_profile_id": "shelf_2",
                "scene_id": 3,
                "shelf_layer": "1",
                "shelf_position": "3",
            },
        )
    assert result["ok"] is False
    assert result["error"] == "shelf_slot_arm_mismatch"
    thread_class.assert_not_called()


def test_large_shelf_full_flow_preview_is_isolated_and_fails_closed():
    test_runtime = profiled_runtime()
    preview, inspection = server.build_full_flow_preview(
        test_runtime,
        {
            "drink_id": "dahongpao_milk_tea",
            "left_shelf_layer": "1",
            "left_shelf_position": "3",
            "shelf_profile_id": "shelf_2",
            "scene_id": 3,
        },
    )
    assert inspection.ready is False
    assert preview["ready"] is False
    assert preview["shelf_profile"]["id"] == "shelf_2"
    assert "shelf_route_not_ready" in {
        issue["code"] for issue in preview["issues"]
    }


def full_flow_job_and_stages(test_runtime):
    inspection = server.inspect_full_flow(
        {
            "drink_id": "dahongpao_milk_tea",
            "left_shelf_layer": "1",
            "left_shelf_position": "1",
            "right_destination": "default",
            "scene_id": 1,
            "gripper_width_m": 0.08,
        },
        drinks=server.DRINKS,
        place_poses=test_runtime.place_poses,
        pick_place_poses=test_runtime.pick_place_poses,
        default_scene_id=1,
        default_gripper_width_m=0.08,
    )
    assert inspection.ready is True
    job = {
        "job_id": "test-full-flow",
        "task_mode": "full_flow",
        "status": "running",
        "phase": "LEFT_RESTOCK",
        "prompt": "Dahongpao Milk Tea",
        "scene_id": 1,
        "gripper_width_m": 0.08,
        "stage_results": [],
        "ros_call_active": False,
    }
    test_runtime.busy = True
    test_runtime.current_job = job
    return job, inspection.stages()


def test_preview_fails_closed_before_ros_when_right_pose_is_missing():
    test_runtime = runtime(full_flow_pose=None)
    preview, inspection = server.build_full_flow_preview(
        test_runtime,
        {
            "drink_id": "dahongpao_milk_tea",
            "left_shelf_layer": "1",
            "left_shelf_position": "1",
        },
    )
    assert inspection.ready is False
    assert preview["ready"] is False
    assert preview["state_machine"]["checked"] is False
    assert "right_place_pose_not_configured" in {
        issue["code"] for issue in preview["issues"]
    }


def test_existing_restock_route_keeps_original_service_and_arm():
    test_runtime = runtime()
    with patch.object(server.threading, "Thread") as thread_class:
        result = server.start_execute_job(
            test_runtime,
            {
                "task_mode": "restock",
                "drink_id": "dahongpao_milk_tea",
                "arm": "left",
                "shelf_layer": "1",
                "shelf_position": "1",
            },
        )
    assert result["ok"] is True
    assert result["job"]["service"] == "/execute_named_grasp_task"
    assert result["job"]["arm"] == "left"
    assert result["job"]["auto_save_recording"] is False
    thread_class.return_value.start.assert_called_once_with()


def test_execute_and_record_job_preserves_auto_save_flag():
    test_runtime = runtime()
    with patch.object(server.threading, "Thread") as thread_class:
        result = server.start_execute_job(
            test_runtime,
            {
                "task_mode": "restock",
                "drink_id": "dahongpao_milk_tea",
                "arm": "right",
                "auto_save_recording": True,
            },
        )
    assert result["ok"] is True
    assert result["job"]["auto_save_recording"] is True
    thread_class.return_value.start.assert_called_once_with()


def test_auto_recording_is_finished_and_saved_after_success():
    test_runtime = runtime()
    recorder = AutoSaveRecorder()
    test_runtime.recorder = recorder
    job = {"auto_save_recording": True, "recording_phase": None}
    test_runtime.current_job = job

    result = server.finalize_recording_for_job(
        test_runtime,
        job,
        {"success": True, "message": "done"},
    )

    assert recorder.finish_calls == [(True, "done")]
    assert recorder.save_calls == ["success"]
    assert result["ok"] is True
    assert result["status"] == "saved"
    assert result["path"] == "/tmp/episode_001"
    assert job["recording_phase"] == "saved"


def test_manual_recording_is_not_auto_saved_after_execution():
    test_runtime = runtime()
    recorder = AutoSaveRecorder()
    test_runtime.recorder = recorder
    job = {"auto_save_recording": False, "recording_phase": None}
    test_runtime.current_job = job

    result = server.finalize_recording_for_job(
        test_runtime,
        job,
        {"success": False, "message": "failed"},
    )

    assert recorder.finish_calls == [(False, "failed")]
    assert recorder.save_calls == []
    assert result is None


def test_auto_recording_quality_failure_stays_rejected():
    test_runtime = runtime()
    recorder = AutoSaveRecorder(
        {
            "ok": False,
            "error": "recording_quality_failed",
            "message": "quality failed",
            "quality": {"ok": False, "failures": ["camera_gap"]},
            "recording": {"status": "rejected"},
        }
    )
    test_runtime.recorder = recorder
    job = {"auto_save_recording": True, "recording_phase": None}
    test_runtime.current_job = job

    result = server.finalize_recording_for_job(
        test_runtime,
        job,
        {"success": True, "message": "done"},
    )

    assert result["ok"] is False
    assert result["status"] == "rejected"
    assert result["quality"]["failures"] == ["camera_gap"]
    assert job["recording_phase"] == "rejected"


def test_existing_pick_route_keeps_original_service_and_arm():
    test_runtime = runtime()
    with patch.object(server.threading, "Thread") as thread_class:
        result = server.start_execute_job(
            test_runtime,
            {
                "task_mode": "pick",
                "drink_id": "dahongpao_milk_tea",
                "arm": "right",
            },
        )
    assert result["ok"] is True
    assert result["job"]["service"] == "/execute_named_pick_task"
    assert result["job"]["arm"] == "right"
    place_pose = thread_class.call_args.kwargs["args"][2]
    assert place_pose.pose.position.z == RIGHT_POSE["position"][2]
    thread_class.return_value.start.assert_called_once_with()


def test_stop_during_full_flow_transition_does_not_poison_ros_cancel_flag():
    test_runtime = runtime()
    test_runtime.busy = True
    test_runtime.current_job = {
        "task_mode": "full_flow",
        "status": "running",
        "phase": "TRANSITION_CHECK",
        "ros_call_active": False,
    }
    result = server.stop_execute_job(test_runtime)
    assert result["ok"] is True
    assert result["accepted"] is True
    assert test_runtime.current_job["status"] == "stopping"
    assert "不会启动" in result["message"]


def test_full_flow_stop_stays_latched_when_cancel_service_is_unavailable():
    test_runtime = runtime()
    test_runtime.cancel_client = UnavailableClient()
    test_runtime.busy = True
    test_runtime.current_job = {
        "task_mode": "full_flow",
        "status": "running",
        "phase": "LEFT_RESTOCK",
        "ros_call_active": True,
    }
    result = server.stop_execute_job(test_runtime)
    assert result["ok"] is False
    assert result["error"] == "cancel_service_unavailable"
    assert test_runtime.current_job["status"] == "stopping"
    assert "已阻止后续阶段" in result["message"]


def test_valid_full_flow_start_is_fixed_left_then_right_without_ros_call_in_http_thread():
    test_runtime = runtime()
    test_runtime.full_flow_restock_client = ReadyClient()
    test_runtime.pick_client = ReadyClient()
    test_runtime.full_flow_pick_client = ReadyClient()
    test_runtime.cancel_client = ReadyClient()
    test_runtime.parameter_client = ReadyParameterClient()
    with patch.object(server.threading, "Thread") as thread_class:
        result = server.start_full_flow_job(
            test_runtime,
            {
                "drink_id": "dahongpao_milk_tea",
                "left_shelf_layer": "1",
                "left_shelf_position": "1",
                "right_destination": "default",
                "scene_id": 1,
                "gripper_width_m": 0.08,
            },
        )
    assert result["ok"] is True
    assert result["preview"]["ready"] is True
    assert result["job"]["phase"] == "LEFT_RESTOCK"
    assert result["job"]["arm"] == "left"
    assert result["job"]["services"] == {
        "left_restock": "/execute_named_full_flow_restock_task",
        "right_pick": "/execute_named_full_flow_pick_task",
        "cancel": "/cancel_named_grasp_task",
    }
    thread_class.return_value.start.assert_called_once_with()


def test_full_flow_worker_sends_fixed_left_then_right_requests():
    test_runtime = runtime()
    left_client = RecordingServiceClient(message="left complete")
    right_client = RecordingServiceClient(message="right complete")
    test_runtime.full_flow_restock_client = left_client
    test_runtime.full_flow_pick_client = right_client
    job, stages = full_flow_job_and_stages(test_runtime)

    server.execute_full_flow_job(test_runtime, job, stages)

    assert len(left_client.calls) == 1
    assert len(right_client.calls) == 1
    assert left_client.calls[0].arm_name == "left"
    assert left_client.calls[0].camera_frame == "cam_left_color_optical_frame"
    assert right_client.calls[0].arm_name == "right"
    assert right_client.calls[0].camera_frame == "cam_right_color_optical_frame"
    assert right_client.calls[0].place_tcp_pose.pose.position.z == FULL_FLOW_RIGHT_POSE["position"][2]
    assert test_runtime.busy is False
    assert test_runtime.last_result["status"] == "completed"
    assert [item["phase"] for item in test_runtime.last_result["stage_results"]] == [
        "LEFT_RESTOCK",
        "RIGHT_PICK",
    ]


def test_left_stage_timeout_calls_cancel_and_never_starts_right():
    test_runtime = runtime()
    left_client = RecordingServiceClient(pending=True)
    right_client = RecordingServiceClient()
    cancel_client = RecordingCancelClient()
    test_runtime.full_flow_restock_client = left_client
    test_runtime.full_flow_pick_client = right_client
    test_runtime.cancel_client = cancel_client
    test_runtime.full_flow_stage_timeout_sec = 0.01
    job, stages = full_flow_job_and_stages(test_runtime)

    server.execute_full_flow_job(test_runtime, job, stages)

    assert len(left_client.calls) == 1
    assert left_client.pending_future.cancel_count == 1
    assert cancel_client.call_count == 1
    assert right_client.calls == []
    assert test_runtime.last_result["status"] == "failed"
    assert test_runtime.last_result["failed_phase"] == "LEFT_RESTOCK"
    assert test_runtime.last_result["error"] == "stage_timeout"
