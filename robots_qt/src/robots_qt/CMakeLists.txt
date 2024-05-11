cmake_minimum_required(VERSION 3.8)
project(robots_qt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(QT NAMES Qt6 COMPONENTS Core Quick Positioning Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Quick Positioning Widgets REQUIRED)
file(GLOB PRO_SOURCE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  src/*.cpp)
# file(GLOB PRO_LAUNCH_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  launch/*.py)
file(GLOB PRO_UI_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  ui/*.ui)
file(GLOB PRO_INCLUDE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  include/*.h *.hpp)
file(GLOB PRO_RESOURCE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  resource/*.qrc)

# set(PRO_MAIN_FILE "src/main.cpp")

qt6_wrap_ui(QT_UI_HPP ${PRO_UI_DIR})
qt6_wrap_cpp(QT_MOC_HPP ${PRO_INCLUDE_DIR})
# qt6_add_resources(QT_RESOUCE_HPP ${PRO_RESOURC11E_DIR})
include_directories(
        include/
        ${CMAKE_CURRENT_BINARY_DIR}
)
add_executable(${PROJECT_NAME}
    ${PRO_SOURCE_DIR}
    # ${PRO_LAUNCH_DIR}
    ${PRO_INCLUDE_DIR}
    ${PRO_UI_DIR}
    ${PRO_RESOURCE_DIR}
    ${QT_UI_HPP}
    ${QT_MOC_HPP}
    # ${QT_RESOUCE_HPP}
  )
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs std_msgs nav_msgs sensor_msgs)

target_link_libraries(${PROJECT_NAME} Qt${QT_VERSION_MAJOR}::Widgets)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY ui resource
   DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
