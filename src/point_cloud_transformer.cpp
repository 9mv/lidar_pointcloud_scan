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

  deletePointsPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

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
  this->declare_parameter<int>("updateBuffer", 10);
  this->declare_parameter<int>("samplesPerAngle", 3);
  this->declare_parameter<float>("xAdjust", 0.0);
  this->declare_parameter<float>("yAdjust", 0.0);
  this->declare_parameter<float>("zAdjust", 0.0);
  this->declare_parameter<std::string>("pcdExportPath", "/tmp/");

  // Get parameters
  this->get_parameter<bool>("appendMode", appendMode_);
  LOG_ROS_INFO(this, "appendMode: %s", appendMode_ ? "true" : "false");

  this->get_parameter_or<int>("updateBuffer", updateBuffer_, 10);
  LOG_ROS_INFO(this, "updateBuffer: %d", updateBuffer_);
  if (updateBuffer_ < 1)
  {
    LOG_ROS_ERROR(this, "Invalid update buffer value: must be positive");
    rclcpp::shutdown();
  }

  int processingType;
  this->get_parameter_or<int>("processingType", processingType, 0);
  processingType_ = static_cast<ProcessingType>(processingType);
  LOG_ROS_INFO(this, "processingType: %d", processingType_);

  this->get_parameter_or<int>("samplesPerAngle", samplesPerAngle_, 2);
  if (samplesPerAngle_ < 1)
  {
    samplesPerAngle_ = 1;
  }
  LOG_ROS_INFO(this, "samplesPerAngle: %d", samplesPerAngle_);

  this->get_parameter_or<float>("xAdjust", xAdjust_, 0.0);
  this->get_parameter_or<float>("yAdjust", yAdjust_, 0.0);
  this->get_parameter_or<float>("zAdjust", zAdjust_, 0.0);
  LOG_ROS_INFO(this, "Angular adjustments: x: %f, y: %f, z: %f", xAdjust_, yAdjust_, zAdjust_);

  this->get_parameter<std::string>("pcdExportPath", pcdExportPath_);
  LOG_ROS_INFO(this, "pcdExportPath: %s", pcdExportPath_.c_str());
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
  firstLaserScan_ = true;
  currentBuffer_ = 0;
  inScan_ = true;

  // Delete points in RViz2
  visualization_msgs::msg::Marker deletePointsMarker;
  deletePointsMarker.action = visualization_msgs::msg::Marker::DELETEALL;
  deletePointsMarker.header.frame_id = "laser";
  deletePointsMarker.header.stamp = this->now();
  deletePointsPublisher_ -> publish(deletePointsMarker);
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

    // Avoid unnecessary scan notififications logs
    if (state_ == TRANSFORMER_READY || state_ == TRANSFORMER_NOT_READY)
    {
        LOG_ROS_INFO(this, "Notifying transformer state %d", static_cast<uint32_t>(state_));
    }
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
    // Publish remaining points
    publishPointCloud();
    dumpScanToFile();

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

Result PointCloudTransformer::dumpScanToFile()
{
  std::string outputPathStr = pcdExportPath_;
  Result res = RESULT_OK;
  if (outputPathStr.empty())
  {
    outputPathStr = "/tmp/";
  }

  rcpputils::fs::path outputPath(outputPathStr);
  if (!rcpputils::fs::exists(outputPath))
  {
    try 
    {
      rcpputils::fs::create_directories(outputPath);
      LOG_ROS_INFO(this, "Created directory for the PointCloud export file: %s", outputPath.string().c_str());
    }
    catch (const std::exception& e)
    {
      LOG_ROS_ERROR(this, "Failed to create directory for the PointCloud export file: %s", outputPath.string().c_str());
      return RESULT_ERROR;
    }
  }

  auto now = std::chrono::system_clock::now();
  auto timeNow = std::chrono::system_clock::to_time_t(now);
  std::tm time = *std::localtime(&timeNow);

  char timeInFile[15];
  std::strftime(timeInFile, sizeof(timeInFile), "%Y%m%d%H%M%S", &time);

  rcpputils::fs::path fullOutputPath = outputPath / ("PointCloudExport_" + std::string(timeInFile) + ".pcd");

  pcl::PointCloud<pcl::PointXYZ> pclPointCloud;
  pcl::fromROSMsg(cumulativePointCloud_, pclPointCloud);

  size_t numPoints = cumulativePointCloud_.width * cumulativePointCloud_.height;
  LOG_ROS_INFO(this, "Dumping point cloud with %ld points", numPoints);

  LOG_ROS_INFO(this, "Generated point cloud contains %lu points (width: %u, height: %u)", 
             pclPointCloud.points.size(), 
             pclPointCloud.width, 
             pclPointCloud.height);

  if (pcl::io::savePCDFileASCII(fullOutputPath.string(), pclPointCloud) == 0) 
  {
    LOG_ROS_INFO(this, "PointCloud successfully saved to %s", fullOutputPath.string().c_str());
  } 
  else
  {
    LOG_ROS_ERROR(this, "Failed to save PointCloud to %s", fullOutputPath.string().c_str());
    res = RESULT_ERROR;
  }
  return res;
}

void PointCloudTransformer::angleUpdateCallback (const lidar_pointcloud_scan::msg::Angle::SharedPtr msg)
{
  if (inScan_)
  {
    currentAngle_ = msg->angle;
    currentSamplesPerAngle_ = 0;
    state_ = TRANSFORMER_WAIT_FOR_SCAN;
    notifyTransformerState();
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

  if (currentSamplesPerAngle_ == samplesPerAngle_ && (state_ == TRANSFORMER_READY || state_ == TRANSFORMER_WAIT_FOR_SCAN))
  {
    //LOG_ROS_INFO(this, "Requesting new angle to servo");
    state_ = TRANSFORMER_REQUEST_NEW_ANGLE;
    notifyTransformerState();
  }
}

void PointCloudTransformer::updatePointCloud(const sensor_msgs::msg::LaserScan::SharedPtr lidar_scan)
{
  if (state_ != TRANSFORMER_READY && state_ != TRANSFORMER_WAIT_FOR_SCAN) // If state_ == TRANSFORMER_REQUEST_NEW_ANGLE there might be servo movement causing distortion. Discard.
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

      applyAngularCalibration(x, adjusted_y, z);

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

    size_t numPoints = partialPointCloud_.width * partialPointCloud_.height;
    LOG_ROS_INFO(this, "Partial point cloud has %ld points", numPoints);

    if (!reset_)
    {
      LOG_ROS_INFO(this, "Appending point cloud data. Current buffer: %d", currentBuffer_);
      // Append pointCloud2 to cumulativePointCloud_
      appendToPointCloud(pointCloud2, partialPointCloud_);
      currentBuffer_++;
    }
    else
    {
      if (firstLaserScan_)
      {
        cumulativePointCloud_ = pointCloud2;
        firstLaserScan_ = false;
      }
      partialPointCloud_ = pointCloud2;
      reset_ = false;
    }

    if (static_cast<int>(currentBuffer_) >= updateBuffer_)
    {
      LOG_ROS_INFO(this, "Publishing point cloud data.");
      publishPointCloud();
      currentBuffer_ = 0;
    }
    currentSamplesPerAngle_++;
  }
}

void PointCloudTransformer::applyAngularCalibration(float& x, float& y, float& z)
{
    // Convert adjustments from degrees to radians
    float xRad = DEG2RAD(xAdjust_);
    float yRad = DEG2RAD(yAdjust_);
    float zRad = DEG2RAD(zAdjust_);

    float xOriginal = x, yOriginal = y, zOriginal = z;

    // Apply Z axis rotation
    x = xOriginal * std::cos(zRad) - yOriginal * std::sin(zRad);
    y = xOriginal * std::sin(zRad) + yOriginal * std::cos(zRad);

    xOriginal = x; yOriginal = y;

    // Apply Y axis rotation
    x = xOriginal * std::cos(yRad) + zOriginal * std::sin(yRad);
    z = -xOriginal * std::sin(yRad) + zOriginal * std::cos(yRad);

    yOriginal = y; zOriginal = z;

    // Apply X axis rotation
    y = yOriginal * std::cos(xRad) - zOriginal * std::sin(xRad);
    z = yOriginal * std::sin(xRad) + zOriginal * std::cos(xRad);
}

void PointCloudTransformer::appendToPointCloud(const PointCloud2& pointCloud2, PointCloud2& appendedPointCloud)
{
  //LOG_ROS_INFO(this, "Appending to existing PointCloud2");

  if (appendedPointCloud.width == 0 || appendedPointCloud.height == 0)
  {
    appendedPointCloud = pointCloud2;
    return;
  }

  if (pointCloud2.point_step != appendedPointCloud.point_step)
  {
    return;
  }

  // Update widths of PC2
  size_t oldWidth = appendedPointCloud.width;
  uint32_t newWidth = oldWidth + pointCloud2.width;

  size_t cumulativePoints = appendedPointCloud.width * appendedPointCloud.height;
  //size_t appendPoints = pointCloud2.width * pointCloud2.height;

  appendedPointCloud.data.resize(appendedPointCloud.data.size() + pointCloud2.data.size());

  std::copy(pointCloud2.data.begin(), pointCloud2.data.end(), appendedPointCloud.data.begin() + appendedPointCloud.point_step * cumulativePoints);

  sensor_msgs::PointCloud2Modifier pclModifier(appendedPointCloud);
  pclModifier.resize(newWidth);

  // If the point clouds are unorganized (height = 1), keep height as 1
  if (appendedPointCloud.height == 1 && pointCloud2.height == 1)
  {
    appendedPointCloud.height = 1;
  }
  else
  {
    // If the point clouds are organized, sum their heights and ensure correct width
    appendedPointCloud.height += pointCloud2.height;
  }

  // Optionally, update the timestamp of the target cloud to the latest one
  appendedPointCloud.header.stamp = pointCloud2.header.stamp;

}

Result PointCloudTransformer::publishPointCloud()
{
  Result result = RESULT_OK;
  //LOG_ROS_INFO(this, "Publishing new pointCloud");

  appendToPointCloud(partialPointCloud_, cumulativePointCloud_);

  if (appendMode_)
  {
    size_t numPoints = cumulativePointCloud_.width * cumulativePointCloud_.height;
    LOG_ROS_INFO(this, "Publishing point cloud of %ld points", numPoints);
    pointCloudPublisher_->publish(cumulativePointCloud_);
  }
  else
  {
    size_t numPoints = partialPointCloud_.width * partialPointCloud_.height;
    LOG_ROS_INFO(this, "Publishing partial point cloud of %ld points", numPoints);
    pointCloudPublisher_->publish(partialPointCloud_);
  }
  reset_ = true;
  return result;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}