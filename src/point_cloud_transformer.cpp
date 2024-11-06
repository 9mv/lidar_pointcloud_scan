#include "lidar_pointcloud_scan/point_cloud_transformer.h"

PointCloudTransformer::PointCloudTransformer()
: Node("point_cloud_transformer")
{
  initParameters();

  // Subscriptions and publishers
  angleSubscription_ = this->create_subscription<lidar_pointcloud_scan::msg::Angle>(
      "tilt_angle", 10, 
      std::bind(&PointCloudTransformer::angleUpdateCallback, this, std::placeholders::_1));

  lidarScanSubscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, 
      std::bind(&PointCloudTransformer::lidarScanCallback, this, std::placeholders::_1));

  pointCloudPublisher_ = this->create_publisher<PointCloud2>("scan_3d", 10);

  // Services
  startScanService_ = this->create_service<lidar_pointcloud_scan::srv::StartScan>(
    "start_scan_transformer",
    std::bind(&PointCloudTransformer::handleStartScan, this, std::placeholders::_1, std::placeholders::_2)
  );
  stopScanService_ = this->create_service<lidar_pointcloud_scan::srv::StopScan>(
    "stop_scan_transformer",
    std::bind(&PointCloudTransformer::handleStopScan, this, std::placeholders::_1, std::placeholders::_2)
  );

  transformerStateClient_ = this->create_client<TransformerState>("transformer_state");
}

void PointCloudTransformer::initParameters ()
{
  LOG_ROS_INFO(this, "Initializing parameters");

  // Declare parameters
  this->declare_parameter<bool>("appendMode", false);
  this->declare_parameter<int>("processingType", 0);

  // Get parameters
  this->get_parameter<bool>("appendMode", appendMode_);
  LOG_ROS_INFO(this, "appendMode: %s", appendMode_ ? "true" : "false");

  int processingType;
  this->get_parameter_or<int>("processingType", processingType, 0);
  processingType_ = static_cast<ProcessingType>(processingType);
  LOG_ROS_INFO(this, "processingType: %d", processingType_);
}

void PointCloudTransformer::handleStartScan(
  const std::shared_ptr<lidar_pointcloud_scan::srv::StartScan::Request> request,
  std::shared_ptr<lidar_pointcloud_scan::srv::StartScan::Response> response)
{
  (void) request;
  if (inScan_)
  {
    LOG_ROS_ERROR(this, "Scan in progress. Skipping handleStartScan.");
    return;
  }

  LOG_ROS_INFO(this, "Start scan requested");

  // @todo -> configure the response
  response->success = true;
  
  // Notify readiness to motor so it starts moving
  state_ = TRANSFORMER_READY;
  notifyTransformerState();

  // @todo -> eliminate current pointCloud data
  reset_ = true;
  inScan_ = true;
}

Result PointCloudTransformer::notifyTransformerState ()
{
    Result res = RESULT_OK;

    // Wait for the service to be available
    if (!transformerStateClient_->wait_for_service(std::chrono::seconds(5))) 
    {
        LOG_ROS_ERROR(this, "Timeout for service available wait exceeded");
        return res;
    }

    auto request = std::make_shared<TransformerState::Request>();
    request->state = static_cast<int8_t>(state_);

    LOG_ROS_INFO(this, "Notifying transformer state %d", static_cast<uint32_t>(state_));
    auto future = transformerStateClient_->async_send_request(
        request, 
        std::bind(&PointCloudTransformer::notifyTransformerStateCallback, this, std::placeholders::_1)
    );

    return res;
}

void PointCloudTransformer::notifyTransformerStateCallback (rclcpp::Client<TransformerState>::SharedFuture transformerStateServiceFuture)
{
  auto response = transformerStateServiceFuture.get();
  if (!response->success)
  {
    LOG_ROS_ERROR(this, "Notification of transformer state failed");
    rclcpp::shutdown();
  }
}

void PointCloudTransformer::handleStopScan(const std::shared_ptr<lidar_pointcloud_scan::srv::StopScan::Request> request,  std::shared_ptr<lidar_pointcloud_scan::srv::StopScan::Response> response)
{
  if (!inScan_)
  {
    LOG_ROS_ERROR(this, "Scan not in progress. Skipping handleStopScan.");
    return;
  }
  auto endScanReason = static_cast<EndScanReason>(request->stop_reason);

  LOG_ROS_INFO(this, "Stop scan requested with reason %d", endScanReason);
  stopScan(endScanReason);

  // @todo -> configure the response
  response->success = true;
}

void PointCloudTransformer::stopScan (EndScanReason reason)
{
  if (!inScan_)
  {
    return;
  }
  LOG_ROS_INFO(this, "End scanning process");

  if (reason == EndScanReason::END)
  {
    LOG_ROS_INFO(this, "Scan COMPLETE");
    
    // @todo -> necessito notificar a algú que ha acabat i com?
  }
  else if (reason == EndScanReason::CANCEL)
  {
    LOG_ROS_INFO(this, "Scan STOPPED");
    reset_ = true;
  }
  state_ = TRANSFORMER_NOT_READY;
  notifyTransformerState();
  inScan_ = false;
}

void PointCloudTransformer::angleUpdateCallback (const lidar_pointcloud_scan::msg::Angle::SharedPtr msg)
{
  if (inScan_)
  {
    currentAngle_ = msg->angle;
  }
}

void PointCloudTransformer::lidarScanCallback (const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  //int count = scan->scan_time / scan->time_increment;

  //LOG_ROS_INFO(this, "New laserscan received: %s[%d]. Adjusting with processingType %d", scan->header.frame_id.c_str(), count, processingType_);
  if (!inScan_)
  {
    return;
  }

  if (processingType_ == POINT_CLOUD_PROCESSING)
  {
    updatePointCloud(scan);
  }
}

void PointCloudTransformer::updatePointCloud(const sensor_msgs::msg::LaserScan::SharedPtr lidar_scan)
{
  if (state_ != TRANSFORMER_READY)
  {
    return;
  }
  //LOG_ROS_INFO(this, "Updating point cloud");
  std::vector<PointCloudPoint> points;
  float motorAngleInRad = DEG2RAD(currentAngle_);
  float angleMin = lidar_scan->angle_min;
  float angleIncrement = lidar_scan->angle_increment;

  // Iterate each of the points given by the scan
  for (size_t i = 0; i < lidar_scan->ranges.size(); i++)
  {
    float range = lidar_scan->ranges[i];

    // Check valid range
    if (std::isfinite(range) && range >= lidar_scan->range_min && range <= lidar_scan->range_max)
    {
      // Current angle is initial angle + angle corresponding to i point
      float angle = angleMin + i * angleIncrement;

      // Get x and y using basic trigonometry given the angle of scan and radius
      float x = range * std::cos(angle);
      float y = range * std::sin(angle);
      
      float adjusted_y = y * std::cos(motorAngleInRad);
      float z = y * std::sin(motorAngleInRad);
      PointCloudPoint point;
      point.x = x;
      point.y = adjusted_y;
      point.z = z;
      
      points.push_back(point);
    }
  }

  //LOG_ROS_INFO(this, "Number of valid points: %zu", points.size());

  // Create and populate the PointCloud2 message
  PointCloud2 pointCloud2;
  pointCloud2.header = lidar_scan->header;
  pointCloud2.height = 1;
  pointCloud2.width = points.size();

  sensor_msgs::PointCloud2Modifier modifier(pointCloud2);
  modifier.setPointCloud2Fields(3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);

  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointCloud2, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointCloud2, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointCloud2, "z");

  for (const auto& point : points)
  {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    ++iter_x; ++iter_y; ++iter_z;
  }
  //LOG_ROS_INFO(this, "Final cumulative PointCloud2 size: %zu", cumulativePointCloud_.width);

  if (inScan_)
  {
    if (!reset_ && appendMode_)
    {
      // Append pointCloud2 to cumulativePointCloud_
      appendToCumulativePointCloud(pointCloud2);
    }
    else
    {
      //LOG_ROS_INFO(this, "Overriding current PointCloud2");
      cumulativePointCloud_ = pointCloud2;
      reset_ = false;
    }
    publishPointCloud();
  }
}

void PointCloudTransformer::appendToCumulativePointCloud(const PointCloud2& pointCloud2)
{
  //LOG_ROS_INFO(this, "Appending to existing PointCloud2");

  if (cumulativePointCloud_.width == 0 || cumulativePointCloud_.height == 0)
  {
    cumulativePointCloud_ = pointCloud2;
    return;
  }

  if (pointCloud2.point_step != cumulativePointCloud_.point_step)
  {
    return;
  }

  // Update widths of PC2
  size_t oldWidth = cumulativePointCloud_.width;
  uint32_t newWidth = oldWidth + pointCloud2.width;

  size_t cumulativePoints = cumulativePointCloud_.width * cumulativePointCloud_.height;
  //size_t appendPoints = pointCloud2.width * pointCloud2.height;

  cumulativePointCloud_.data.resize(cumulativePointCloud_.data.size() + pointCloud2.data.size());

  std::copy(pointCloud2.data.begin(), pointCloud2.data.end(), cumulativePointCloud_.data.begin() + cumulativePointCloud_.point_step * cumulativePoints);

  sensor_msgs::PointCloud2Modifier pclModifier(cumulativePointCloud_);
  pclModifier.resize(newWidth);

    // If the point clouds are unorganized (height = 1), keep height as 1
  if (cumulativePointCloud_.height == 1 && pointCloud2.height == 1)
  {
    cumulativePointCloud_.height = 1;
  }
  else
  {
    // If the point clouds are organized, sum their heights and ensure correct width
    cumulativePointCloud_.height += pointCloud2.height;
  }

  // Optionally, update the timestamp of the target cloud to the latest one
  cumulativePointCloud_.header.stamp = pointCloud2.header.stamp;

}

Result PointCloudTransformer::publishPointCloud()
{
  Result result = RESULT_OK;
  //LOG_ROS_INFO(this, "Publishing new pointCloud");
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