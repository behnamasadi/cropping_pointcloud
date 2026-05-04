#!/usr/bin/env bash
# Run Gazebo (server-only, headless) and the ros_gz bridge as the two long-lived
# processes in this container. Either dying brings the container down so docker
# compose can restart it cleanly.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"

: "${GZ_CMD:?must be set (ign gazebo or gz sim)}"
: "${GZ_WORLD:=gpu_lidar_sensor.sdf}"
: "${GZ_TOPIC:=/lidar/points}"
: "${ROS_TOPIC:=/camera/rgb/points}"

# If DISPLAY is set, run Gazebo with GUI; otherwise headless (-s).
# -r starts the sim immediately.
GUI_FLAG=""
if [ -z "${DISPLAY:-}" ]; then GUI_FLAG="-s"; fi
${GZ_CMD} ${GUI_FLAG} -r "${GZ_WORLD}" &
GZ_PID=$!

# Give Gazebo a moment to bind its transport before the bridge subscribes.
sleep 3

ros2 run ros_gz_bridge parameter_bridge \
  "${GZ_TOPIC}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  --ros-args -r "${GZ_TOPIC}:=${ROS_TOPIC}" &
BRIDGE_PID=$!

trap 'kill -TERM $GZ_PID $BRIDGE_PID 2>/dev/null || true' TERM INT

# Exit when the first of the two dies.
wait -n "$GZ_PID" "$BRIDGE_PID"
EXIT_CODE=$?

kill -TERM "$GZ_PID" "$BRIDGE_PID" 2>/dev/null || true
wait "$GZ_PID" "$BRIDGE_PID" 2>/dev/null || true
exit "$EXIT_CODE"
