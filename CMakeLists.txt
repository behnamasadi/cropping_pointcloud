cmake_minimum_required(VERSION 3.10)
project(cropping_pointcloud)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)

# ── Shared library that registers a composable node ────────────────────────
# Loadable by ComposableNodeContainer. Enables intra-process zero-copy when
# the publisher (e.g. the PMD driver) lives in the same container with
# `use_intra_process_comms=True`.
add_library(cropping_pointcloud_component SHARED src/cropping_pointcloud.cpp)
target_include_directories(cropping_pointcloud_component PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(cropping_pointcloud_component
  PRIVATE ${PCL_DEFINITIONS} CROPPING_POINTCLOUD_BUILD_AS_COMPONENT_ONLY)
target_link_libraries(cropping_pointcloud_component ${PCL_LIBRARIES})
ament_target_dependencies(cropping_pointcloud_component
  rclcpp rclcpp_components sensor_msgs visualization_msgs pcl_conversions)
rclcpp_components_register_nodes(cropping_pointcloud_component "CroppingPointCloud")

# ── Standalone executable (backwards-compatible launch) ────────────────────
# Same source compiled as a binary for users who prefer `ros2 run` /
# `ros2 launch cropping_demo.launch.py` without composition.
add_executable(cropping_pointcloud src/cropping_pointcloud.cpp)
target_include_directories(cropping_pointcloud PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(cropping_pointcloud PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(cropping_pointcloud ${PCL_LIBRARIES})
ament_target_dependencies(cropping_pointcloud
  rclcpp rclcpp_components sensor_msgs visualization_msgs pcl_conversions)

install(TARGETS cropping_pointcloud_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS cropping_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
