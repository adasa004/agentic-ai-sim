#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION="lot10-sensors-$$"
export ROS_DOMAIN_ID="${LOT10_ROS_DOMAIN_ID:-88}"

python3 make_world.py
gz sim -r lot10_prototype.sdf &
sim_pid=$!
cleanup() {
    kill "$sim_pid" ${bridge_pid:+"$bridge_pid"} ${guard_pid:+"$guard_pid"} ${odom_pid:+"$odom_pid"} 2>/dev/null || true
    wait "$sim_pid" ${bridge_pid:+"$bridge_pid"} ${guard_pid:+"$guard_pid"} ${odom_pid:+"$odom_pid"} 2>/dev/null || true
}
trap cleanup EXIT

/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge /world/lot10_prototype/set_pose@ros_gz_interfaces/srv/SetEntityPose --ros-args -p config_file:="$PWD/bridge.yaml" &
bridge_pid=$!
guard_options=()
if [[ ${1:-} == --manual ]]; then
    guard_options=(--ros-args -p recovery_enabled:=false)
fi
python3 obstacle_guard.py "${guard_options[@]}" &
guard_pid=$!
python3 encoder_gyro_pose.py &
odom_pid=$!
echo "For the camera in another ROS terminal, export ROS_DOMAIN_ID=$ROS_DOMAIN_ID."
if [[ ${1:-} == --manual ]]; then
    echo "Keep this terminal focused to drive. i = forward, j/l = turn, k = stop."
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_requested
else
    python3 patrol.py
fi
