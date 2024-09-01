#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lidar_pointcloud_scan/msg/angle.hpp"
#include "lidar_pointcloud_scan/types.h"

class PointCloudTransformer : public rclcpp::Node
{
public:
  PointCloudTransformer();

// Private methods
private:
    void angle_update_callback (const lidar_pointcloud_scan::msg::Angle::SharedPtr msg);

// Private attributes
private:
  rclcpp::Subscription<lidar_pointcloud_scan::msg::Angle>::SharedPtr subscription_;
};