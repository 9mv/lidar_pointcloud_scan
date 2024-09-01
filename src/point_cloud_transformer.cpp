#include "lidar_pointcloud_scan/point_cloud_transformer.h"

PointCloudTransformer::PointCloudTransformer()
: Node("point_cloud_transformer")
{
    subscription_ = this->create_subscription<lidar_pointcloud_scan::msg::Angle>(
        "tilt_angle", 10, 
        std::bind(&PointCloudTransformer::angle_update_callback, this, std::placeholders::_1));
}


void PointCloudTransformer::angle_update_callback (const lidar_pointcloud_scan::msg::Angle::SharedPtr msg)
{
    LOG_INFO(this, "New angle received: '%d'", msg->angle);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}