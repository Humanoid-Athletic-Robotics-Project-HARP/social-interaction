// Exercise the built STVL buffer without rclcpp::init(), nodes, or DDS.
#include <cstdlib>
#include <iostream>
#include <memory>

#include "rcl/time.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "spatio_temporal_voxel_layer/measurement_buffer.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  const double deadline = std::stod(argv[1]);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  if (rcl_enable_ros_time_override(clock->get_clock_handle()) != RCL_RET_OK) {return 2;}
  auto at = [&](double seconds) {
      return rcl_set_ros_time_override(
        clock->get_clock_handle(), static_cast<int64_t>(seconds * 1e9)) == RCL_RET_OK;
    };
  if (!at(10.0)) {return 2;}
  tf2_ros::Buffer tf(clock);
  // Every lookup below is identity, supplied synchronously without a listener.
  tf.setUsingDedicatedThread(true);
  buffer::MeasurementBuffer buffer(
    "lidar_marking", "/cloud", 0.0, deadline, 0.08, 1.8, 3.0, tf, "odom", "",
    0.5, 0.0, 3.0, 1.815, 0.05, 6.29, 10.0, true, false, 0.05,
    buffer::Filters::PASSTHROUGH, 0, true, true, ModelType::THREE_DIMENSIONAL_LIDAR,
    clock, rclcpp::get_logger("stvl_watchdog_test"));

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "odom";  // Identity TF needs no listener or ROS node.
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(0);
  cloud.header.stamp = clock->now();
  buffer.BufferROSCloud(cloud);
  if (!buffer.UpdatedAtExpectedRate()) {return 3;}

  // A normal 10 Hz cloud stream remains current; missing clouds expire.
  if (!at(10.1) || !buffer.UpdatedAtExpectedRate()) {return 4;}
  if (!at(12.0) || buffer.UpdatedAtExpectedRate()) {
    std::cerr << "Missing lidar still reports current after 2 seconds\n";
    return 5;
  }

  // An empty cloud restores freshness, even when there is no obstacle to mark.
  cloud.header.stamp = clock->now();
  buffer.BufferROSCloud(cloud);
  if (!buffer.UpdatedAtExpectedRate()) {return 6;}

  // A cloud whose points are all above the height filter also counts as data.
  if (!at(14.0) || buffer.UpdatedAtExpectedRate()) {return 7;}
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  *x = 1.0F; *y = 0.0F; *z = 100.0F;
  cloud.header.stamp = clock->now();
  buffer.BufferROSCloud(cloud);
  if (!buffer.UpdatedAtExpectedRate()) {return 8;}
  return 0;
}
