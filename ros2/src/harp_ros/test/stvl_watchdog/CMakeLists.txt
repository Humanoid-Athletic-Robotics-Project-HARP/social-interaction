cmake_minimum_required(VERSION 3.16)
project(harp_stvl_watchdog LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(spatio_temporal_voxel_layer REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters common)
find_package(Boost REQUIRED COMPONENTS thread)
find_library(STVL_CORE spatio_temporal_voxel_layer_core
  HINTS "${spatio_temporal_voxel_layer_DIR}/../../../lib" REQUIRED)

add_executable(stvl_watchdog watchdog.cpp)
target_compile_features(stvl_watchdog PRIVATE cxx_std_17)
target_include_directories(stvl_watchdog PRIVATE "${STVL_SOURCE}/include" ${PCL_INCLUDE_DIRS})
target_link_libraries(stvl_watchdog ${STVL_CORE} ${PCL_LIBRARIES} Boost::thread)
ament_target_dependencies(stvl_watchdog spatio_temporal_voxel_layer rclcpp rclcpp_lifecycle sensor_msgs
  tf2_ros pcl_conversions)
