#include "lidar_to_grid/lidar_to_grid_node.hpp"

LidarToGridNode::LidarToGridNode()
: Node("lidar_to_grid_node")
{
  // Declare and get parameters
  this->declare_parameter("grid_resolution", 0.1);  // 10cm per cell
  
  this->declare_parameter("grid_width", 50.0);      // 50m width
  this->declare_parameter("grid_height", 50.0);     // 50m height
  this->declare_parameter("min_height", 0.1);       // 10cm min height
  
  this->declare_parameter("max_height", 2.0);       // 2m max height
  this->declare_parameter("lidar_topic", "/carla/ego_vehicle/lidar");
  this->declare_parameter("grid_frame_id", "map");

  grid_resolution_ = this->get_parameter("grid_resolution").as_double();
  grid_width_ = this->get_parameter("grid_width").as_double();
  grid_height_ = this->get_parameter("grid_height").as_double();
  
  min_height_ = this->get_parameter("min_height").as_double();
  max_height_ = this->get_parameter("max_height").as_double();
  lidar_topic_ = this->get_parameter("lidar_topic").as_string();
  grid_frame_id_ = this->get_parameter("grid_frame_id").as_string();

  // Initialize TF listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create publisher and subscriber
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "occupancy_grid", 10);
  
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_, 10,
    std::bind(&LidarToGridNode::lidarCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "LidarToGridNode initialized");
}

void LidarToGridNode::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert ROS message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // Create and publish occupancy grid
  auto grid = createOccupancyGrid(*cloud);
  grid.header.stamp = this->now();
  grid.header.frame_id = grid_frame_id_;
  grid_pub_->publish(grid);
}

nav_msgs::msg::OccupancyGrid LidarToGridNode::createOccupancyGrid(
  const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  nav_msgs::msg::OccupancyGrid grid;

  // Set grid metadata
  grid.info.resolution = grid_resolution_;
  grid.info.width = static_cast<unsigned int>(grid_width_ / grid_resolution_);
  grid.info.height = static_cast<unsigned int>(grid_height_ / grid_resolution_);
  
  // Set grid origin at the center
  grid.info.origin.position.x = -grid_width_ / 2.0;
  grid.info.origin.position.y = -grid_height_ / 2.0;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  // Initialize grid data
  grid.data.resize(grid.info.width * grid.info.height, -1);  // -1 for unknown

  // Process point cloud
  for (const auto& point : cloud.points) {
    // Skip points outside height range
    if (point.z < min_height_ || point.z > max_height_) {
      continue;
    }

    // Convert point to grid coordinates
    int grid_x = static_cast<int>((point.x - grid.info.origin.position.x) / grid_resolution_);
    int grid_y = static_cast<int>((point.y - grid.info.origin.position.y) / grid_resolution_);

    // Check if point is within grid bounds
    if (grid_x >= 0 && grid_x < static_cast<int>(grid.info.width) &&
        grid_y >= 0 && grid_y < static_cast<int>(grid.info.height)) {
      // Mark cell as occupied (100 for occupied)
      grid.data[grid_y * grid.info.width + grid_x] = 100;
    }
  }

  return grid;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarToGridNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 