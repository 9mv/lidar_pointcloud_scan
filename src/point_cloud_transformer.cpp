#include "lidar_pointcloud_scan/point_cloud_transformer.h"

PointCloudTransformer::PointCloudTransformer()
: Node("point_cloud_transformer")
{
  angleSubscription_ = this->create_subscription<lidar_pointcloud_scan::msg::Angle>(
      "tilt_angle", 10, 
      std::bind(&PointCloudTransformer::angleUpdateCallback, this, std::placeholders::_1));

  lidarScanSubscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, 
      std::bind(&PointCloudTransformer::lidarScanCallback, this, std::placeholders::_1));

  pointCloudPublisher_ = this->create_publisher<PointCloud2>("scan_3d", 10);

}

void PointCloudTransformer::angleUpdateCallback (const lidar_pointcloud_scan::msg::Angle::SharedPtr msg)
{
  LOG_INFO(this, "New angle received: '%d'", msg->angle);
  currentAngle_ = msg->angle;
}

void PointCloudTransformer::lidarScanCallback (const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  int count = scan->scan_time / scan->time_increment;

  updatePointCloud(scan);

  LOG_INFO(this, "New laserscan received: %s[%d]:", scan->header.frame_id.c_str(), count);
}

void PointCloudTransformer::updatePointCloud(const sensor_msgs::msg::LaserScan::SharedPtr lidar_scan)
{
  // TODO
  publishPointCloud();
}

Result PointCloudTransformer::publishPointCloud()
{
  Result result = RESULT_OK;
  LOG_INFO(this, "publishing new pointCloud");
  pointCloudPublisher_->publish(cumulativePointCloud_);

  return result;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}