cmake_minimum_required(VERSION 3.8)
project(turtlebot3_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create the executable for check_turtlebot3_node
add_executable(check_turtlebot3_node src/check_turtlebot3_node.cpp)

# Set include directories
target_include_directories(check_turtlebot3_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Set dependencies for the executable
ament_target_dependencies(check_turtlebot3_node  # 使用正确的目标名
  "rclcpp"
  "std_msgs"
)

# Install the executable
install(TARGETS 
  check_turtlebot3_node
  DESTINATION lib/${PROJECT_NAME})  # 安装到包的 lib 目录

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Testing (if required)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyrght_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()  # 结束配置