cmake_minimum_required(VERSION 3.5)
project(nav2_graceful_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(angles REQUIRED)
find_package(visualization_msgs REQUIRED)

# HARP (Foxy port): Foxy's nav2_package() defaults to C++14; the control law uses std::clamp.
set(CMAKE_CXX_STANDARD 17)
nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  tf2_geometry_msgs
  nav_2d_utils
  angles
  visualization_msgs
)

# Add Smooth Control Law as library
add_library(smooth_control_law SHARED src/smooth_control_law.cpp)
ament_target_dependencies(smooth_control_law ${dependencies})

# Add Graceful Controller
set(library_name nav2_graceful_controller)

add_library(${library_name} SHARED
  src/graceful_controller.cpp
  src/parameter_handler.cpp
  src/path_handler.cpp
  src/utils.cpp
)

target_link_libraries(${library_name} smooth_control_law)
ament_target_dependencies(${library_name} ${dependencies})

install(TARGETS smooth_control_law ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)


ament_export_include_directories(include)
ament_export_libraries(smooth_control_law ${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core graceful_controller_plugin.xml)

ament_package()
