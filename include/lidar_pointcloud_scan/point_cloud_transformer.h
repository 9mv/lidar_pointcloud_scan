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

struct PointCloudPoint {
  float x;
  float y;
  float z;
};

enum EndScanReason {
  CANCEL,
  END
};

enum ProcessingType {
  POINT_CLOUD_PROCESSING,
  LASER_SCAN_PROCESSING
};

class PointCloudTransformer : public rclcpp::Node
{
public:
  PointCloudTransformer();

// Private methods
private:
    void initParameters();

    // Point Cloud processing
    void updatePointCloud(const sensor_msgs::msg::LaserScan::SharedPtr lidar_scan);
    void appendToCumulativePointCloud(const PointCloud2 & pointCloud);
    Result publishPointCloud();

    // Subscription callbacks
    void angleUpdateCallback (const lidar_pointcloud_scan::msg::Angle::SharedPtr msg);
    void lidarScanCallback (const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Init/Stop scan callbacks
    void initScanCallback ();
    void stopScanCallback (EndScanReason reason);

// Private attributes
private:
  // Subscriptions
  rclcpp::Subscription<lidar_pointcloud_scan::msg::Angle>::SharedPtr angleSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidarScanSubscription_;

  // Publishers
  rclcpp::Publisher<PointCloud2>::SharedPtr pointCloudPublisher_;

  // set to true while there is a scan in process
  bool inScan_ = false;

  // If reset is true, the point cloud is reseted
  bool reset_ = false;

  // Current angle of the lidar
  int currentAngle_ = -90.0;

  // Append mode will accumulate the point cloud
  // instead of replacing it
  bool appendMode_ = true;

  // Publish LaserScan or PointCloud events
  ProcessingType processingType_ = LASER_SCAN_PROCESSING;

  // Point cloud
  PointCloud2 cumulativePointCloud_;
};