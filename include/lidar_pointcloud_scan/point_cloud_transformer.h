#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "lidar_pointcloud_scan/msg/angle.hpp"
#include "lidar_pointcloud_scan/srv/start_scan.hpp"
#include "lidar_pointcloud_scan/srv/stop_scan.hpp"
#include "lidar_pointcloud_scan/srv/transformer_state.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <math.h>
#include "lidar_pointcloud_scan/types.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*M_PI/180.)

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using TransformerState = lidar_pointcloud_scan::srv::TransformerState;

class PointCloudTransformer : public rclcpp::Node
{
public:
  PointCloudTransformer();

// Private methods
private:
    /*
    * Get parameters for the node
    */
    void initParameters();

    /*
    * Point Cloud processing
    */ 
    void updatePointCloud(const sensor_msgs::msg::LaserScan::SharedPtr lidar_scan);
    void appendToCumulativePointCloud(const PointCloud2 & pointCloud);
    Result publishPointCloud();

    /*
    * Subscription callbacks
    */
    void angleUpdateCallback (const lidar_pointcloud_scan::msg::Angle::SharedPtr msg);
    void lidarScanCallback (const sensor_msgs::msg::LaserScan::SharedPtr scan);


    /*
    * Handle the service request to start the scanning process.
    * @param request: request message, empty.
    * @param response: response success of the operation.
    */
    void handleStartScan (const std::shared_ptr<lidar_pointcloud_scan::srv::StartScan::Request> request, std::shared_ptr<lidar_pointcloud_scan::srv::StartScan::Response> response);

    /*
    * Start the scanning process.
    */
    void startScan ();

    /*
    * Handle the service request to stop the scanning process.
    * @param request: request message, including the EndScanReason for the stop scan.
    * @param response: response success of the operation.
    */
    void handleStopScan (const std::shared_ptr<lidar_pointcloud_scan::srv::StopScan::Request> request, std::shared_ptr<lidar_pointcloud_scan::srv::StopScan::Response> response);

    /*
    * Stop the scanning process.
    * @param reason: reason for stopping the scan -> @todo: think about this. The reason will be passed as a member of the message type of the topic or will it be passed as argument?
    */
    void stopScan (EndScanReason reason);

    /*
    * Dump the current scan to a point cloud file.
    * @param filePath: path to the file where the scan will be saved.
    * @return: RESULT_OK if the file is saved successfully, RESULT_ERROR otherwise.
    */
    Result dumpScanToFile(std::string filePath);

    // Transformer state service
    /*
    * Notify the transformer state to the rotation motor.
    * @return: result of the operation.
    */
    Result notifyTransformerState();
    void notifyTransformerStateCallback (rclcpp::Client<TransformerState>::SharedFuture transformerStateServiceFuture);


// Private attributes
private:
  // Subscriptions
  rclcpp::Subscription<lidar_pointcloud_scan::msg::Angle>::SharedPtr angleSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidarScanSubscription_;

  // Publishers
  rclcpp::Publisher<PointCloud2>::SharedPtr pointCloudPublisher_;

  // Services
  rclcpp::Client<TransformerState>::SharedPtr transformerStateClient_;
  rclcpp::Service<lidar_pointcloud_scan::srv::StartScan>::SharedPtr startScanService_;
  rclcpp::Service<lidar_pointcloud_scan::srv::StopScan>::SharedPtr stopScanService_;

  // set to true while there is a scan in process
  bool inScan_ = false;

  // If reset is true, the point cloud is reseted
  bool reset_ = false;

  // Current angle of the lidar
  float currentAngle_ = -90.0;

  // Append mode will accumulate the point cloud
  // instead of replacing it
  bool appendMode_ = true;

  // PointCloud publish buffer of scans
  int updateBuffer_ = 50;
  
  // Current buffer number of scans
  uint32_t currentBuffer_ = 0;

  // Publish LaserScan or PointCloud events
  ProcessingType processingType_ = LASER_SCAN_PROCESSING;

  // Point cloud
  PointCloud2 cumulativePointCloud_;

  // Internal readiness state
  PointCloudTransformerState state_ = TRANSFORMER_NOT_READY;
};