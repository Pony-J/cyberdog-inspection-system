#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/cyberdog_ws}"
PKG_INSTALL_SHARE_DIR="${PKG_INSTALL_SHARE_DIR:-$WORKSPACE_DIR/install/cyberdog_ai_pkg/share/cyberdog_ai_pkg}"
PKG_INSTALL_LIB_DIR="${PKG_INSTALL_LIB_DIR:-$WORKSPACE_DIR/install/cyberdog_ai_pkg/lib}"
PKG_INSTALL_LIBEXEC_DIR="${PKG_INSTALL_LIBEXEC_DIR:-$PKG_INSTALL_LIB_DIR/cyberdog_ai_pkg}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/mi1035085/camera/color/image_raw}"
DEPTH_IMAGE_TOPIC="${DEPTH_IMAGE_TOPIC:-/mi1035085/camera/aligned_depth_to_color/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/mi1035085/camera/color/camera_info}"
BODY_TOPIC="${BODY_TOPIC:-/mi1035085/body}"
METER_TOPIC="${METER_TOPIC:-/mi1035085/meter}"
FACE_TOPIC="${FACE_TOPIC:-/mi1035085/face}"
FIRE_TOPIC="${FIRE_TOPIC:-/mi1035085/fire_alarm}"
FALL_TOPIC="${FALL_TOPIC:-/mi1035085/fall_alarm}"
HAT_TOPIC="${HAT_TOPIC:-/mi1035085/hat_alarm}"
SMOKE_TOPIC="${SMOKE_TOPIC:-/mi1035085/smoke_alarm}"
GATHER_TOPIC="${GATHER_TOPIC:-/mi1035085/gather_alarm}"
CAMERA_ENABLE_SERVICE="${CAMERA_ENABLE_SERVICE:-/mi1035085/camera/enable}"
WS_PORT="${WS_PORT:-9091}"
RUNTIME_STATUS_TOPIC="${RUNTIME_STATUS_TOPIC:-/vision/runtime_status}"
ANNOTATED_FPS="${ANNOTATED_FPS:-5.0}"
BODY_PROCESS_EVERY_N_FRAMES="${BODY_PROCESS_EVERY_N_FRAMES:-3}"
SMOKE_ROI_EXPAND_RATIO="${SMOKE_ROI_EXPAND_RATIO:-0.0}"
SMOKE_UPPER_BODY_HEIGHT_RATIO="${SMOKE_UPPER_BODY_HEIGHT_RATIO:-1.0}"
SMOKE_LOG_CLASSIFICATION_DETAILS="${SMOKE_LOG_CLASSIFICATION_DETAILS:-1}"
SMOKE_NORMALIZATION_MODE="${SMOKE_NORMALIZATION_MODE:-imagenet}"
SMOKE_OUTPUT_POSTPROCESS="${SMOKE_OUTPUT_POSTPROCESS:-softmax}"
SMOKE_CENTER_CROP_SQUARE="${SMOKE_CENTER_CROP_SQUARE:-0}"
SMOKE_MIMIC_X86_DOUBLE_COLOR_CONVERT="${SMOKE_MIMIC_X86_DOUBLE_COLOR_CONVERT:-0}"

ENABLE_CAMERA="${ENABLE_CAMERA:-1}"
ENABLE_BODY_DETECTOR="${ENABLE_BODY_DETECTOR:-1}"
ENABLE_METER_READING="${ENABLE_METER_READING:-0}"
ENABLE_FACE_DETECTOR="${ENABLE_FACE_DETECTOR:-0}"
ENABLE_FIRE_ALARM="${ENABLE_FIRE_ALARM:-1}"
ENABLE_FALL_ALARM="${ENABLE_FALL_ALARM:-1}"
ENABLE_HAT_ALARM="${ENABLE_HAT_ALARM:-0}"
ENABLE_SMOKE_ALARM="${ENABLE_SMOKE_ALARM:-0}"
ENABLE_GATHER_ALARM="${ENABLE_GATHER_ALARM:-0}"
STOP_OLD_PROCESSES="${STOP_OLD_PROCESSES:-1}"

PIDS=()

log() {
  printf '[start_vision_stack] %s\n' "$*"
}

cleanup() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  wait >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

is_enabled() {
  [[ "${1}" == "1" || "${1}" == "true" ]]
}

prepend_path_once() {
  local var_name="$1"
  local path_value="$2"
  [[ -n "$path_value" && -d "$path_value" ]] || return 0
  local current="${!var_name:-}"
  case ":${current}:" in
    *:"${path_value}":*) ;;
    *)
      if [[ -n "$current" ]]; then
        printf -v "$var_name" '%s:%s' "$path_value" "$current"
      else
        printf -v "$var_name" '%s' "$path_value"
      fi
      export "$var_name"
      ;;
  esac
}

repair_runtime_env() {
  prepend_path_once COLCON_PREFIX_PATH "$WORKSPACE_DIR/install"
  prepend_path_once AMENT_PREFIX_PATH "$WORKSPACE_DIR/install"
  prepend_path_once CMAKE_PREFIX_PATH "$WORKSPACE_DIR/install"
  prepend_path_once LD_LIBRARY_PATH "$PKG_INSTALL_LIB_DIR"
  prepend_path_once PATH "$PKG_INSTALL_LIBEXEC_DIR"

  local python_dir=""
  if [[ -d "$WORKSPACE_DIR/install/cyberdog_ai_pkg/local/lib/python3.10/dist-packages" ]]; then
    python_dir="$WORKSPACE_DIR/install/cyberdog_ai_pkg/local/lib/python3.10/dist-packages"
  elif [[ -d "$WORKSPACE_DIR/install/cyberdog_ai_pkg/local/lib/python3.8/dist-packages" ]]; then
    python_dir="$WORKSPACE_DIR/install/cyberdog_ai_pkg/local/lib/python3.8/dist-packages"
  fi
  prepend_path_once PYTHONPATH "$python_dir"
}

pkg_exec_path() {
  local exec_name="$1"
  local candidate="$PKG_INSTALL_LIBEXEC_DIR/$exec_name"
  if [[ -e "$candidate" ]]; then
    printf '%s\n' "$candidate"
    return 0
  fi
  return 1
}

pkg_launch_path() {
  local launch_name="$1"
  local candidate="$PKG_INSTALL_SHARE_DIR/launch/$launch_name"
  if [[ -e "$candidate" ]]; then
    printf '%s\n' "$candidate"
    return 0
  fi
  return 1
}

run_pkg_exec() {
  local exec_name="$1"
  shift
  local exec_path=""
  if exec_path="$(pkg_exec_path "$exec_name")"; then
    "$exec_path" "$@"
  else
    ros2 run cyberdog_ai_pkg "$exec_name" "$@"
  fi
}

run_pkg_launch() {
  local launch_name="$1"
  shift
  local launch_path=""
  if launch_path="$(pkg_launch_path "$launch_name")"; then
    ros2 launch "$launch_path" "$@"
  else
    ros2 launch cyberdog_ai_pkg "$launch_name" "$@"
  fi
}

ensure_entrypoint_permissions() {
  local script
  for script in \
    "$WORKSPACE_DIR/src/cyberdog_ai_pkg/scripts/vision_manager_node.py" \
    "$WORKSPACE_DIR/src/cyberdog_ai_pkg/scripts/vision_runtime_manager.py" \
    "$WORKSPACE_DIR/src/cyberdog_ai_pkg/scripts/vision_ws_gateway.py" \
    "$WORKSPACE_DIR/src/cyberdog_ai_pkg/scripts/publish_test_image.py" \
    "$WORKSPACE_DIR/src/cyberdog_ai_pkg/scripts/start_vision_stack.sh"; do
    if [[ -f "$script" && ! -x "$script" ]]; then
      chmod +x "$script" 2>/dev/null || true
    fi
  done
}

ensure_detector_engine() {
  local detector_name="$1"
  local engine_rel="$2"
  local engine_path="$PKG_INSTALL_SHARE_DIR/$engine_rel"

  if [[ -f "$engine_path" ]]; then
    return 0
  fi

  log "Disabling $detector_name for this run: missing engine $engine_path"
  return 1
}

prepare_detector_runtime() {
  local enable_var_name="$1"
  local detector_name="$2"
  local engine_rel="$3"
  local enable_value="${!enable_var_name}"

  if ! is_enabled "$enable_value"; then
    return 0
  fi

  if ensure_detector_engine "$detector_name" "$engine_rel"; then
    return 0
  fi

  log "Disabling $detector_name for this run because its engine is unavailable"
  printf -v "$enable_var_name" '%s' "0"
}

print_help() {
  cat <<'EOF'
Usage:
  ros2 run cyberdog_ai_pkg start_vision_stack

Optional environment variables:
  ENABLE_CAMERA=1
  ENABLE_BODY_DETECTOR=1
  ENABLE_METER_READING=0
  ENABLE_FACE_DETECTOR=0
  ENABLE_FIRE_ALARM=0
  ENABLE_FALL_ALARM=1
  ENABLE_HAT_ALARM=0
  ENABLE_SMOKE_ALARM=0
  ENABLE_GATHER_ALARM=0
  STOP_OLD_PROCESSES=1
  ANNOTATED_FPS=5.0
  BODY_PROCESS_EVERY_N_FRAMES=3
  SMOKE_ROI_EXPAND_RATIO=0.0
  SMOKE_UPPER_BODY_HEIGHT_RATIO=1.0
  SMOKE_LOG_CLASSIFICATION_DETAILS=1
  SMOKE_NORMALIZATION_MODE=imagenet
  SMOKE_OUTPUT_POSTPROCESS=softmax
  SMOKE_CENTER_CROP_SQUARE=0
  SMOKE_MIMIC_X86_DOUBLE_COLOR_CONVERT=0

Examples:
  ros2 run cyberdog_ai_pkg start_vision_stack
  ENABLE_METER_READING=1 ros2 run cyberdog_ai_pkg start_vision_stack
  ENABLE_FIRE_ALARM=1 ros2 run cyberdog_ai_pkg start_vision_stack
  ENABLE_FALL_ALARM=1 ros2 run cyberdog_ai_pkg start_vision_stack
  ENABLE_HAT_ALARM=1 ros2 run cyberdog_ai_pkg start_vision_stack
  ENABLE_GATHER_ALARM=1 ros2 run cyberdog_ai_pkg start_vision_stack
  BODY_PROCESS_EVERY_N_FRAMES=3 ros2 run cyberdog_ai_pkg start_vision_stack
  ENABLE_SMOKE_ALARM=1 ros2 run cyberdog_ai_pkg start_vision_stack
  ENABLE_FALL_ALARM=1 ENABLE_HAT_ALARM=1 ENABLE_GATHER_ALARM=1 ros2 run cyberdog_ai_pkg start_vision_stack
  ENABLE_SMOKE_ALARM=1 SMOKE_UPPER_BODY_HEIGHT_RATIO=0.75 ros2 run cyberdog_ai_pkg start_vision_stack
  ENABLE_SMOKE_ALARM=1 SMOKE_OUTPUT_POSTPROCESS=softmax ros2 run cyberdog_ai_pkg start_vision_stack
  ENABLE_FACE_DETECTOR=1 ENABLE_METER_READING=1 ros2 run cyberdog_ai_pkg start_vision_stack

Notes:
  - This script is the one-key dog-side starter for camera + runtime manager +
    optional static detectors + vision_manager + WS gateway.
  - It never builds TensorRT engines at startup. If an engine is missing,
    that detector is skipped for the initial runtime apply.
  - vision_ws_gateway.launch.py is kept as a gateway-only debug entry.
  - Runtime-managed detectors: person, fire_alarm, fall_alarm, hat_alarm,
    gather_alarm.
  - Future algorithm services can be added in start_optional_algorithms().
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  print_help
  exit 0
fi

source_env() {
  # ROS/COLCON setup scripts sometimes read unset helper vars.
  # Temporarily disable nounset while sourcing them.
  set +u
  if [[ -f /etc/mi/mi_config ]]; then
    # shellcheck disable=SC1091
    source /etc/mi/mi_config
  elif [[ -f /opt/ros2/cyberdog/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros2/cyberdog/setup.bash
  fi

  if [[ -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "$WORKSPACE_DIR/install/setup.bash"
  else
    set -u
    log "Missing workspace setup: $WORKSPACE_DIR/install/setup.bash"
    exit 1
  fi
  set -u
  repair_runtime_env
}

stop_old_processes() {
  if [[ "$STOP_OLD_PROCESSES" != "1" ]]; then
    return
  fi

  log "Stopping stale vision processes"
  pkill -f 'vision_ws_gateway.py' 2>/dev/null || true
  pkill -f 'vision_manager_node.py' 2>/dev/null || true
  pkill -f 'vision_runtime_manager.py' 2>/dev/null || true
  pkill -f 'body_detector_node' 2>/dev/null || true
  pkill -f 'fire_detector_node' 2>/dev/null || true
  pkill -f 'smoke_detector_node' 2>/dev/null || true
  pkill -f 'gather_detector_node' 2>/dev/null || true
  pkill -f 'meter_reading_node' 2>/dev/null || true
  pkill -f 'face_detector_node' 2>/dev/null || true
  pkill -f 'vision_manager.launch.py' 2>/dev/null || true
  pkill -f 'vision_ws_gateway.launch.py' 2>/dev/null || true
  fuser -k "${WS_PORT}/tcp" >/dev/null 2>&1 || true
  sleep 1
}

enable_camera_stream() {
  if ! is_enabled "$ENABLE_CAMERA"; then
    return
  fi

  if ! timeout 3 ros2 service type "$CAMERA_ENABLE_SERVICE" >/dev/null 2>&1; then
    log "Camera enable service $CAMERA_ENABLE_SERVICE not available, continuing"
    return
  fi

  log "Enabling camera stream via $CAMERA_ENABLE_SERVICE"
  local output=""
  if output=$(timeout 8 ros2 service call \
    "$CAMERA_ENABLE_SERVICE" std_srvs/srv/SetBool "{data: true}" 2>&1); then
    printf '%s\n' "$output"
  else
    log "Camera enable call timed out or failed, continuing"
    printf '%s\n' "$output"
  fi
  sleep 1
}

start_bg() {
  local name="$1"
  shift
  local startup_wait_sec="${STARTUP_WAIT_SEC:-1}"
  if [[ "${1:-}" == "--startup-wait" ]]; then
    startup_wait_sec="$2"
    shift 2
  fi
  log "Starting $name"
  "$@" &
  local pid="$!"
  PIDS+=("$pid")
  sleep "$startup_wait_sec"
  if ! kill -0 "$pid" 2>/dev/null; then
    wait "$pid" || true
    log "$name exited during startup"
    exit 1
  fi
}

start_body_detector() {
  return 0
}

join_list_brackets() {
  local IFS=,
  printf '[%s]' "$*"
}

start_fire_detector() {
  return 0
}

start_fall_detector() {
  return 0
}

start_hat_detector() {
  return 0
}

start_smoke_detector() {
  if ! is_enabled "$ENABLE_SMOKE_ALARM"; then
    return
  fi

  start_bg \
    smoke_detector \
    --startup-wait 3 \
    run_pkg_exec smoke_detector_node --ros-args \
      -p "input_image_topic:=${IMAGE_TOPIC}" \
      -p "output_detection_topic:=${SMOKE_TOPIC}" \
      -p "detector_name:=smoke_alarm" \
      -p "roi_expand_ratio:=${SMOKE_ROI_EXPAND_RATIO}" \
      -p "upper_body_height_ratio:=${SMOKE_UPPER_BODY_HEIGHT_RATIO}" \
      -p "log_classification_details:=$([[ "${SMOKE_LOG_CLASSIFICATION_DETAILS}" == "1" ]] && echo true || echo false)" \
      -p "normalization_mode:=${SMOKE_NORMALIZATION_MODE}" \
      -p "output_postprocess:=${SMOKE_OUTPUT_POSTPROCESS}" \
      -p "center_crop_square:=$([[ "${SMOKE_CENTER_CROP_SQUARE}" == "1" ]] && echo true || echo false)" \
      -p "mimic_x86_double_color_convert:=$([[ "${SMOKE_MIMIC_X86_DOUBLE_COLOR_CONVERT}" == "1" ]] && echo true || echo false)"
}

start_gather_detector() {
  if ! is_enabled "$ENABLE_GATHER_ALARM"; then
    return
  fi

  start_bg \
    gather_detector \
    --startup-wait 2 \
    run_pkg_exec gather_detector_node --ros-args \
      -p "body_topic:=${BODY_TOPIC}" \
      -p "output_detection_topic:=${GATHER_TOPIC}" \
      -p "detector_name:=gather_alarm" \
      -p "process_every_n_frames:=3" \
      -p "log_detection_summary:=true"
}

start_meter_reading() {
  if ! is_enabled "$ENABLE_METER_READING"; then
    return
  fi

  start_bg \
    meter_reading \
    run_pkg_exec meter_reading_node --ros-args \
      -p "input_image_topic:=${IMAGE_TOPIC}" \
      -p "depth_image_topic:=${DEPTH_IMAGE_TOPIC}" \
      -p "camera_info_topic:=${CAMERA_INFO_TOPIC}" \
      -p "output_meter_topic:=${METER_TOPIC}" \
      -p "log_detection_summary:=true"
}

start_face_detector() {
  if ! is_enabled "$ENABLE_FACE_DETECTOR"; then
    return
  fi

  start_bg \
    face_detector \
    run_pkg_exec face_detector_node --ros-args \
      -p "input_image_topic:=${IMAGE_TOPIC}" \
      -p "output_face_topic:=${FACE_TOPIC}" \
      -p "log_detection_summary:=true"
}

start_optional_algorithms() {
  # Future dog-side algorithms can be added here as extra background jobs.
  # Keep them separate from the core chain so frontend issues can be debugged
  # without guessing which optional service changed the behavior.
  start_face_detector
  start_meter_reading
}

build_static_enabled_detectors() {
  local enabled=()

  if is_enabled "$ENABLE_METER_READING"; then
    enabled+=(meter)
  fi

  if is_enabled "$ENABLE_SMOKE_ALARM"; then
    enabled+=(smoke_alarm)
  fi

  printf '%s\n' "${enabled[@]}"
}

build_initial_runtime_detectors() {
  local enabled=()

  if is_enabled "$ENABLE_BODY_DETECTOR"; then
    enabled+=(person)
  fi

  if is_enabled "$ENABLE_FIRE_ALARM"; then
    enabled+=(fire_alarm)
  fi

  if is_enabled "$ENABLE_FALL_ALARM"; then
    enabled+=(fall_alarm)
  fi

  if is_enabled "$ENABLE_HAT_ALARM"; then
    enabled+=(hat_alarm)
  fi

  if is_enabled "$ENABLE_GATHER_ALARM"; then
    enabled+=(gather_alarm)
  fi

  printf '%s\n' "${enabled[@]}"
}

start_runtime_manager() {
  local static_enabled=()
  while IFS= read -r detector_name; do
    [[ -n "$detector_name" ]] && static_enabled+=("$detector_name")
  done < <(build_static_enabled_detectors)

  start_bg \
    vision_runtime_manager \
    run_pkg_exec vision_runtime_manager.py --ros-args \
      -p "pkg_install_share_dir:=${PKG_INSTALL_SHARE_DIR}" \
      -p "workspace_dir:=${WORKSPACE_DIR}" \
      -p "pkg_install_lib_dir:=${PKG_INSTALL_LIB_DIR}" \
      -p "pkg_install_libexec_dir:=${PKG_INSTALL_LIBEXEC_DIR}" \
      -p "image_topic:=${IMAGE_TOPIC}" \
      -p "body_topic:=${BODY_TOPIC}" \
      -p "fire_topic:=${FIRE_TOPIC}" \
      -p "fall_topic:=${FALL_TOPIC}" \
      -p "hat_topic:=${HAT_TOPIC}" \
      -p "gather_topic:=${GATHER_TOPIC}" \
      -p "body_process_every_n_frames:=${BODY_PROCESS_EVERY_N_FRAMES}" \
      -p "meter_topic:=${METER_TOPIC}" \
      -p "depth_image_topic:=${DEPTH_IMAGE_TOPIC}" \
      -p "camera_info_topic:=${CAMERA_INFO_TOPIC}" \
      -p "meter_process_every_n_frames:=3" \
      -p "runtime_status_topic:=${RUNTIME_STATUS_TOPIC}" \
      -p "static_enabled_detectors:=$(join_list_brackets "${static_enabled[@]}")"
}

start_vision_manager() {
  local enabled=()
  local available=(person meter fire_alarm fall_alarm hat_alarm smoke_alarm gather_alarm)
  local extra_topics=(
    "fire_alarm=${FIRE_TOPIC}"
    "fall_alarm=${FALL_TOPIC}"
    "hat_alarm=${HAT_TOPIC}"
    "smoke_alarm=${SMOKE_TOPIC}"
    "gather_alarm=${GATHER_TOPIC}"
  )

  if is_enabled "$ENABLE_METER_READING"; then
    enabled+=(meter)
  fi

  if is_enabled "$ENABLE_SMOKE_ALARM"; then
    enabled+=(smoke_alarm)
  fi

  start_bg \
    vision_manager \
    run_pkg_exec vision_manager_node.py --ros-args \
      -p "image_topic:=${IMAGE_TOPIC}" \
      -p "body_topic:=${BODY_TOPIC}" \
      -p "meter_topic:=${METER_TOPIC}" \
      -p "available_detectors:=$(join_list_brackets "${available[@]}")" \
      -p "enabled_detectors:=$(join_list_brackets "${enabled[@]}")" \
      -p "extra_detection_topics:=$(join_list_brackets "${extra_topics[@]}")" \
      -p "annotated_ws_fps:=${ANNOTATED_FPS}" \
      -p "alarm_ws_topic:=${ALARM_WS_TOPIC:-/vision/alarm_ws}"
}

start_gateway() {
  start_bg \
    vision_ws_gateway \
    run_pkg_exec vision_ws_gateway.py --ros-args \
      -p "detections_topic:=/vision/detections" \
      -p "status_topic:=/vision/status" \
      -p "runtime_status_topic:=${RUNTIME_STATUS_TOPIC}" \
      -p "include_annotated:=true" \
      -p "ws_port:=${WS_PORT}"
}

wait_for_runtime_service() {
  local deadline=$((SECONDS + 15))
  while (( SECONDS < deadline )); do
    if timeout 2 ros2 service type /vision/runtime/set_detectors >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

apply_initial_runtime_detectors() {
  local desired=()
  while IFS= read -r detector_name; do
    [[ -n "$detector_name" ]] && desired+=("$detector_name")
  done < <(build_initial_runtime_detectors)

  if ! wait_for_runtime_service; then
    log "Runtime service /vision/runtime/set_detectors not available, skipping initial runtime apply"
    return 0
  fi

  local detectors_yaml="[]"
  if ((${#desired[@]} > 0)); then
    local quoted=()
    local detector_name
    for detector_name in "${desired[@]}"; do
      quoted+=("'${detector_name}'")
    done
    local IFS=,
    detectors_yaml="[${quoted[*]}]"
  fi

  log "Applying initial runtime detectors: ${detectors_yaml}"
  local output=""
  if output=$(timeout 12 ros2 service call \
    /vision/runtime/set_detectors \
    cyberdog_ai_pkg/srv/RuntimeSetDetectors \
    "{detectors: ${detectors_yaml}}" 2>&1); then
    printf '%s\n' "$output"
  else
    log "Initial runtime apply failed, continuing with infrastructure only"
    printf '%s\n' "$output"
  fi
}

main() {
  source_env
  ensure_entrypoint_permissions
  prepare_detector_runtime ENABLE_FIRE_ALARM fire_alarm assets/onnx/fire_250527.engine
  prepare_detector_runtime ENABLE_FALL_ALARM fall_alarm assets/onnx/fall_250325.engine
  prepare_detector_runtime ENABLE_HAT_ALARM hat_alarm assets/onnx/hat_250613.engine
  prepare_detector_runtime ENABLE_SMOKE_ALARM smoke_alarm assets/onnx/smoke_250613.engine
  # gather_alarm reuses body_detector output — no separate engine needed
  stop_old_processes
  enable_camera_stream
  start_runtime_manager
  start_smoke_detector
  start_optional_algorithms
  start_vision_manager
  start_gateway
  apply_initial_runtime_detectors

  log "Vision stack started. Ctrl-C to stop all child processes."
  wait -n "${PIDS[@]}"
}

main "$@"
