cmake_minimum_required(VERSION 2.8.3)
project(cropping_pointcloud)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf dynamic_reconfigure)

generate_dynamic_reconfigure_options( cfg/croppingCloud.cfg )
catkin_package()
#set(PCL_DIR "$ENV{HOME}/usr/share/pcl-1.8/")
MESSAGE("PCL_DIR:" ${PCL_DIR} )
#find_package(PCL 1.8.0 REQUIRED COMPONENTS common octree io kdtree search sample_consensus filters 2d features registration geometry visualization outofcore surface keypoints ml segmentation recognition people tracking stereo)


find_package(PCL 1.7 REQUIRED COMPONENTS common octree io kdtree search sample_consensus filters 2d  geometry visualization  surface segmentation)


MESSAGE("PCL_FOUND:" ${PCL_FOUND})
MESSAGE("PCL_INCLUDE_DIRS:" ${PCL_LIBRARY_DIRS})
MESSAGE("PCL_LIBRARIES:" ${PCL_LIBRARIES})
MESSAGE("PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
MESSAGE("PCL_VERSION:" ${PCL_VERSION})
MESSAGE("PCL_COMPONENTS:" ${PCL_COMPONENTS})
MESSAGE("PCL_DEFINITIONS:" ${PCL_DEFINITIONS})


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


MESSAGE("######################################################################PCL#####################################################################")
MESSAGE("PCL_FOUND:" ${PCL_FOUND})
MESSAGE("PCL_INCLUDE_DIRS:" ${PCL_LIBRARY_DIRS})
MESSAGE("PCL_LIBRARIES:" ${PCL_LIBRARIES})
MESSAGE("PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
MESSAGE("PCL_VERSION:" ${PCL_VERSION})
MESSAGE("PCL_COMPONENTS:" ${PCL_COMPONENTS})
MESSAGE("PCL_DEFINITIONS:" ${PCL_DEFINITIONS})
pkg_check_modules(OpenNI REQUIRED libopenni)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})


MESSAGE("######################################################HDF5 FLANN#############################################################")

find_package(HDF5 REQUIRED )
add_definitions(${HDF5_DEFINITIONS})
include_directories(${HDF5_INCLUDE_DIRS})
link_directories(${HDF5_LIBRARIES})

MESSAGE("HDF5_DEFINITIONS:" ${HDF5_DEFINITIONS})
MESSAGE("HDF5_INCLUDE_DIRS:" ${HDF5_INCLUDE_DIRS})
MESSAGE("HDF5_LIBRARIES:" ${HDF5_LIBRARIES})



include_directories(  ${catkin_INCLUDE_DIRS})




add_executable(cropping_pointcloud src/cropping_pointcloud.cpp)
target_link_libraries(cropping_pointcloud ${catkin_LIBRARIES}  ${PCL_LIBRARIES}   ${PCL_TRACKING_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  ${PCL_SEARCH_LIBRARIES}  ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_FILTERS_LIBRARIES}    ${PCL_APPS_LIBRARIES})


