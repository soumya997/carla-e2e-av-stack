#ifndef LIDAR_TO_GRID_NODE_HPP_
#define LIDAR_TO_GRID_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class LidarToGridNode : public rclcpp::Node
{
public:
  LidarToGridNode();

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  nav_msgs::msg::OccupancyGrid createOccupancyGrid(const pcl::PointCloud<pcl::PointXYZ>& cloud);

  // ROS parameters
  double grid_resolution_;  // meters per cell
  double grid_width_;      // meters
  double grid_height_;     // meters
  double min_height_;      // minimum height to consider points
  double max_height_;      // maximum height to consider points
  std::string lidar_topic_;
  std::string grid_frame_id_;

  // ROS publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif  // LIDAR_TO_GRID_NODE_HPP_ 