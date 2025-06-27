cmake_minimum_required(VERSION 3.5)
project(ros2_monitor_srvs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetTopicInfo.srv"
  "srv/GetGraphInfo.srv"
)

ament_package() 