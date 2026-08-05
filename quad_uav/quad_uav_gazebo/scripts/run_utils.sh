#!/usr/bin/env bash

set -Eeuo pipefail

STARTUP_DELAY_SEC="${STARTUP_DELAY_SEC:-2.0}"
PARAM_RETRIES="${PARAM_RETRIES:-5}"
PARAM_RETRY_DELAY_SEC="${PARAM_RETRY_DELAY_SEC:-1.0}"
NODE_PIDS=()

[[ "$STARTUP_DELAY_SEC" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
    echo "Error: STARTUP_DELAY_SEC 必须是非负数" >&2
    exit 2
}
[[ "$PARAM_RETRIES" =~ ^[1-9][0-9]*$ ]] || {
    echo "Error: PARAM_RETRIES 必须是正整数" >&2
    exit 2
}
[[ "$PARAM_RETRY_DELAY_SEC" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
    echo "Error: PARAM_RETRY_DELAY_SEC 必须是非负数" >&2
    exit 2
}

cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM
    for pid in "${NODE_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    for pid in "${NODE_PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    exit "$exit_code"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

set_px4_parameter() {
    local name="$1"
    local value="$2"
    local attempt
    local output

    for ((attempt = 1; attempt <= PARAM_RETRIES; attempt++)); do
        if output="$(rosrun mavros mavparam set "$name" "$value" 2>&1)"; then
            echo ">> PX4 参数: $name=$value"
            return 0
        fi
        echo ">> 等待设置 $name，重试 $attempt/$PARAM_RETRIES" >&2
        if ((attempt < PARAM_RETRIES)); then
            sleep "$PARAM_RETRY_DELAY_SEC"
        fi
    done

    echo "Warning: 无法设置 $name=$value；PX4 可能已保存该参数，继续启动节点。" >&2
    echo "$output" >&2
    return 0
}

start_node() {
    local label="$1"
    shift
    echo ">> 启动 $label"
    "$@" &
    NODE_PIDS+=("$!")
}

# Configure PX4 v1.15 to fuse external-vision position, velocity and yaw.
set_px4_parameter EKF2_EV_CTRL 15
set_px4_parameter EKF2_HGT_REF 3
set_px4_parameter EKF2_GPS_CTRL 0


start_node "Gazebo 真值转 PX4" rosrun quad_uav_gazebo gt_to_px4.py "$@"
sleep "$STARTUP_DELAY_SEC"
start_node "机体系速度转世界系 odom" rosrun quad_uav_gazebo vel_to_world.py
sleep "$STARTUP_DELAY_SEC"
start_node "点云转世界系" rosrun quad_uav_gazebo cloud_to_world.py

echo ">> 全部节点已启动；按 Ctrl+C 统一退出"
wait -n "${NODE_PIDS[@]}"
