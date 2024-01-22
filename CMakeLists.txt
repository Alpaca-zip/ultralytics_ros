cmake_minimum_required(VERSION 3.8)
project(ultralytics_ros)
set(CMAKE_CXX_STANDARD 17)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(message_filters REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YoloResult.msg"
  DEPENDENCIES
  sensor_msgs
  std_msgs
  vision_msgs
)

include_directories(include)

add_executable(tracker_with_cloud_node src/tracker_with_cloud_node.cpp)
ament_target_dependencies(tracker_with_cloud_node
  rclcpp
  sensor_msgs
  message_filters
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tracker_with_cloud_node "${cpp_typesupport_target}")

install(TARGETS
  tracker_with_cloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  script/tracker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
