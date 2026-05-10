# The cropper unit: node + RViz + rqt + rqt_reconfigure, all in one container.
# Uses osrf/ros:*-desktop-full as the base because it already ships PCL,
# pcl_conversions, RViz2, and the perception meta-package — so the only apt
# install we need is rqt_reconfigure, and the build is just `colcon build`.
ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop-full

ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rqt-reconfigure \
      ros-${ROS_DISTRO}-rqt-graph \
 && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=${ROS_DISTRO}
WORKDIR /ws

# .dockerignore strips build/, install/, log/, .git, .env, README.md, images/
COPY . /ws/src/cropping_pointcloud

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
             colcon build --packages-select cropping_pointcloud --symlink-install"

COPY docker/cropper-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "cropping_pointcloud", "cropping_demo.launch.py"]
