# The "publisher" container: Gazebo (Fortress or Harmonic) + ros_gz_bridge.
# Emits a sensor_msgs/PointCloud2 on /camera/rgb/points via the bridge.
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ARG GZ_PKG=ignition-fortress
ARG DEBIAN_FRONTEND=noninteractive

# OSRF Gazebo apt repo + Gazebo + ros_gz bridge in one apt transaction.
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg lsb-release ca-certificates \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz-bridge \
      ros-${ROS_DISTRO}-ros-gz-sim \
      ${GZ_PKG} \
 && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=${ROS_DISTRO}

COPY docker/gazebo-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
