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
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)

add_executable(cropping_pointcloud src/cropping_pointcloud.cpp)
target_include_directories(cropping_pointcloud PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(cropping_pointcloud PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(cropping_pointcloud ${PCL_LIBRARIES})
ament_target_dependencies(cropping_pointcloud rclcpp sensor_msgs pcl_conversions)

install(TARGETS cropping_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
