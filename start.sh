#!/usr/bin/env bash
set -euo pipefail

WEB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="${DRINK_GRASP_WEB_PID_FILE:-${WEB_DIR}/web.pid}"
LOG_FILE="${DRINK_GRASP_WEB_LOG_FILE:-${WEB_DIR}/web.log}"

set +u
source "${DRINK_GRASP_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
source "${DRINK_GRASP_BRIDGE_SETUP:?Set DRINK_GRASP_BRIDGE_SETUP to the grasp bridge install/setup.bash}"
source "${DRINK_GRASP_PLANNING_SETUP:?Set DRINK_GRASP_PLANNING_SETUP to the planning service install/setup.bash}"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export DRINK_GRASP_HOST="${DRINK_GRASP_HOST:-0.0.0.0}"
export DRINK_GRASP_PORT="${DRINK_GRASP_PORT:-8090}"
export DRINK_GRASP_SERVICE="${DRINK_GRASP_SERVICE:-/execute_named_grasp_task}"
export DRINK_GRASP_PICK_SERVICE="${DRINK_GRASP_PICK_SERVICE:-/execute_named_pick_task}"
export DRINK_GRASP_FULL_FLOW_RESTOCK_SERVICE="${DRINK_GRASP_FULL_FLOW_RESTOCK_SERVICE:-/execute_named_full_flow_restock_task}"
export DRINK_GRASP_FULL_FLOW_PICK_SERVICE="${DRINK_GRASP_FULL_FLOW_PICK_SERVICE:-/execute_named_full_flow_pick_task}"
export DRINK_GRASP_CANCEL_SERVICE="${DRINK_GRASP_CANCEL_SERVICE:-/cancel_named_grasp_task}"
export DRINK_GRASP_SCENE_ID="${DRINK_GRASP_SCENE_ID:-3}"
export DRINK_GRASP_GRIPPER_WIDTH_M="${DRINK_GRASP_GRIPPER_WIDTH_M:-0.08}"
export DRINK_GRASP_CALL_TIMEOUT_SEC="${DRINK_GRASP_CALL_TIMEOUT_SEC:-240}"
export DRINK_GRASP_FULL_FLOW_STAGE_TIMEOUT_SEC="${DRINK_GRASP_FULL_FLOW_STAGE_TIMEOUT_SEC:-300}"
export DRINK_GRASP_FULL_FLOW_TOTAL_TIMEOUT_SEC="${DRINK_GRASP_FULL_FLOW_TOTAL_TIMEOUT_SEC:-650}"
export DRINK_GRASP_STATE_MACHINE_NODE="${DRINK_GRASP_STATE_MACHINE_NODE:-/grasp_bridge_state_machine}"
export DRINK_GRASP_OBJECT_MODE="${DRINK_GRASP_OBJECT_MODE:-mesh}"
export DRINK_GRASP_LIVE_CAMERAS_ENABLED="${DRINK_GRASP_LIVE_CAMERAS_ENABLED:-1}"
export DRINK_GRASP_CAM_HIGH_TOPIC="${DRINK_GRASP_CAM_HIGH_TOPIC:-/cam_high/color/image_raw/compressed}"
export DRINK_GRASP_CAM_LEFT_TOPIC="${DRINK_GRASP_CAM_LEFT_TOPIC:-/cam_left/color/image_raw/compressed}"
export DRINK_GRASP_CAM_RIGHT_TOPIC="${DRINK_GRASP_CAM_RIGHT_TOPIC:-/cam_right/color/image_raw/compressed}"
export DRINK_GRASP_RECORDING_ENABLED="${DRINK_GRASP_RECORDING_ENABLED:-1}"
export DRINK_GRASP_RECORD_DIR="${DRINK_GRASP_RECORD_DIR:-${WEB_DIR}/recordings}"
export DRINK_GRASP_RECORD_STORAGE="${DRINK_GRASP_RECORD_STORAGE:-mcap}"
export DRINK_GRASP_RECORD_STORAGE_PRESET="${DRINK_GRASP_RECORD_STORAGE_PRESET:-zstd_fast}"
export DRINK_GRASP_RECORD_MAX_CACHE_SIZE="${DRINK_GRASP_RECORD_MAX_CACHE_SIZE:-1073741824}"
export DRINK_GRASP_RECORD_READY_TIMEOUT_SEC="${DRINK_GRASP_RECORD_READY_TIMEOUT_SEC:-10}"
export DRINK_GRASP_RECORD_CAMERA_WINDOW_SEC="${DRINK_GRASP_RECORD_CAMERA_WINDOW_SEC:-3}"
export DRINK_GRASP_RECORD_CAMERA_MIN_FPS="${DRINK_GRASP_RECORD_CAMERA_MIN_FPS:-29}"
export DRINK_GRASP_RECORD_CAMERA_MAX_GAP_MS="${DRINK_GRASP_RECORD_CAMERA_MAX_GAP_MS:-50}"
export DRINK_GRASP_RECORD_CAMERA_MAX_DELTA_MS="${DRINK_GRASP_RECORD_CAMERA_MAX_DELTA_MS:-25}"
export DRINK_GRASP_RECORD_CONTROL_HZ="${DRINK_GRASP_RECORD_CONTROL_HZ:-30}"
export DRINK_GRASP_RECORD_CAMERA_MAX_AGE_MS="${DRINK_GRASP_RECORD_CAMERA_MAX_AGE_MS:-100}"
export DRINK_GRASP_RECORD_TOPIC_MAX_AGE_MS="${DRINK_GRASP_RECORD_TOPIC_MAX_AGE_MS:-2000}"
export DRINK_GRASP_RECORD_TOPICS="${DRINK_GRASP_RECORD_TOPICS:-/cam_high/color/image_raw /cam_high/color/camera_info /cam_left/color/image_raw /cam_left/color/camera_info /cam_right/color/image_raw /cam_right/color/camera_info /joint_left /joint_right /joint_states_left /joint_states_right /joint_states /joint_ctrl_cmd_left /joint_ctrl_cmd_right /end_pose_stamped_left /end_pose_stamped_right /arm_status_left /arm_status_right /tf /tf_static /act/episode_event}"

if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(tr -d '[:space:]' < "${PID_FILE}")"
  if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "Drink grasp web already running, pid=${old_pid}"
    echo "URL: http://127.0.0.1:${DRINK_GRASP_PORT}/"
    exit 0
  fi
fi

cd "${WEB_DIR}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"
if command -v setsid >/dev/null 2>&1; then
  nohup setsid /usr/bin/python3 "${WEB_DIR}/server.py" > "${LOG_FILE}" 2>&1 < /dev/null 9>&- &
else
  nohup /usr/bin/python3 "${WEB_DIR}/server.py" > "${LOG_FILE}" 2>&1 < /dev/null 9>&- &
fi
pid="$!"
echo "${pid}" > "${PID_FILE}"

for _ in $(seq 1 30); do
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "Drink grasp web failed to start. Log: ${LOG_FILE}" >&2
    exit 1
  fi
  listener_info="$(ss -ltnp 2>/dev/null | grep ":${DRINK_GRASP_PORT} " || true)"
  if [[ "${listener_info}" == *"pid=${pid},"* ]] \
      && curl -fsS --max-time 1 "http://127.0.0.1:${DRINK_GRASP_PORT}/api/cameras" \
        | grep -q '"integrated":true'; then
    echo "Started drink grasp web, pid=${pid}"
    echo "URL: http://127.0.0.1:${DRINK_GRASP_PORT}/"
    echo "Log: ${LOG_FILE}"
    exit 0
  fi
  sleep 1
done

echo "Drink grasp web start timed out. Log: ${LOG_FILE}" >&2
exit 1
