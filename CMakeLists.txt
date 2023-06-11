cmake_minimum_required(VERSION 3.0.2)
project(ultralytics_ros)
set(CMAKE_CXX_STANDARD 14)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  ros_numpy
  pcl_ros
  tf2_ros
  std_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  visualization_msgs
  image_geometry
  image_transport
  message_filters
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    pcl_ros
    tf2_ros
    std_msgs
    sensor_msgs
    geometry_msgs
    vision_msgs
    visualization_msgs
    image_geometry
    image_transport
    message_filters
  DEPENDS
    PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(tracker_with_cloud_node src/tracker_with_cloud_node.cpp)
target_link_libraries(tracker_with_cloud_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
