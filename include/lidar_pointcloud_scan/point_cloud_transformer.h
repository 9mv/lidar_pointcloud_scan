#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "lidar_pointcloud_scan/msg/angle.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <math.h>
#include "lidar_pointcloud_scan/types.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*M_PI/180.)

using PointCloud2 = sensor_msgs::msg::PointCloud2;

class PointCloudTransformer : public rclcpp::Node
{
public:
  PointCloudTransformer();

// Private methods
private:
    void angleUpdateCallback (const lidar_pointcloud_scan::msg::Angle::SharedPtr msg);
    void lidarScanCallback (const sensor_msgs::msg::LaserScan::SharedPtr scan);

    void updatePointCloud(const sensor_msgs::msg::LaserScan::SharedPtr lidar_scan);
    Result publishPointCloud();

// Private attributes
private:
  rclcpp::Subscription<lidar_pointcloud_scan::msg::Angle>::SharedPtr angleSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidarScanSubscription_;

  rclcpp::Publisher<PointCloud2>::SharedPtr pointCloudPublisher_;

  // set to true while there is a scan in process
  bool inScan_ = false;

  // If reset is true, the point cloud is reseted
  bool reset_ = false;

  // Current angle of the lidar
  int currentAngle_ = 0.0;

  // Point cloud
  PointCloud2 cumulativePointCloud_;
};