cmake_minimum_required(VERSION 3.5)
project(deliverbot_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(deliverbot_msgs
    "msg/Battery.msg"
    "msg/Imu.msg"
    "msg/Velocities.msg"
    DEPENDENCIES geometry_msgs
    )
ament_package()
