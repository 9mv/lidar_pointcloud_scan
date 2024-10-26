#include "lidar_pointcloud_scan/lidar_rotation_motor.h"


LidarRotationMotor::LidarRotationMotor()
: Node("lidar_rotation_motor"), currentAngle_(-90)
{
  initParameters();

  publisher_ = this->create_publisher<lidar_pointcloud_scan::msg::Angle>("tilt_angle", 10); // https://docs.ros2.org/galactic/api/std_msgs/index-msg.html

  stopScanServiceClient_ = this->create_client<lidar_pointcloud_scan::srv::StopScan>("stop_scan");
  Result res = initializeMotor();
  if (res != Result::RESULT_OK)
  {
    LOG_ROS_ERROR(this, "failed to initialize motor");
    rclcpp::shutdown();
    return;
  }

  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(incrementPeriod_)), std::bind(&LidarRotationMotor::timer_callback, this));
}

LidarRotationMotor::~LidarRotationMotor()
{
  stopMotor();
  delete pwmController_;
}

void LidarRotationMotor::initParameters ()
{
  LOG_ROS_INFO(this, "Initializing parameters");

  // Declare parameters
  this->declare_parameter<bool>("motorFakeMode", false);
  this->declare_parameter<float>("angleIncrement", 0.0);
  this->declare_parameter<float>("incrementPeriod", 0.0);
  this->declare_parameter<bool>("calibrationMode", false);
  this->declare_parameter<int>("pwmFrequency", 150);
  this->declare_parameter<float>("angleRange", ANGLE_RANGE);
  this->declare_parameter<int>("sweepsPerScan", 1);

  // Get parameters
  this->get_parameter<bool>("motorFakeMode", motorFakeMode_);
  LOG_ROS_INFO(this, "motorFakeMode: %s", motorFakeMode_ ? "true" : "false");

  this->get_parameter_or<float>("angleIncrement", angleIncrement_, 1.0);
  LOG_ROS_INFO(this, "angleIncrement: %f", angleIncrement_);

  this->get_parameter_or<float>("incrementPeriod", incrementPeriod_, 1);
  LOG_ROS_INFO(this, "incrementPeriod: %f ms", incrementPeriod_);

  this->get_parameter<bool>("calibrationMode", calibrationMode_);
  LOG_ROS_INFO(this, "calibrationMode: %s", calibrationMode_ ? "true" : "false");

  this->get_parameter_or<int>("pwmFrequency", pwmFrequency_, 150);

  this->get_parameter_or<int>("sweepsPerScan", sweepsPerScan_, 1);
  
  ServoMotorRange range = getPwmRange();
  minPwm_ = range.min;
  maxPwm_ = range.max;
  LOG_ROS_INFO(this, "minPwm: %d ms", minPwm_);
  LOG_ROS_INFO(this, "maxPwm: %d ms", maxPwm_);

  float angleRange;
  this->get_parameter_or<float>("angleRange", angleRange, 180.0);
  minAngle_ = -angleRange / 2.0;
  maxAngle_ = angleRange / 2.0;
  LOG_ROS_INFO(this, "minAngle: %f", minAngle_);
  LOG_ROS_INFO(this, "maxAngle: %f", maxAngle_);
}

void LidarRotationMotor::timer_callback()
{
  if (motorState_ == MotorState::UNINITIALIZED || !inScan_)
  {
    return;
  }
  auto message = lidar_pointcloud_scan::msg::Angle();
  message.angle = currentAngle_;
  updateAngle();
  publisher_->publish(message);

  //@todo -> this function should not manage the real motor behavior -> maintain only for fake mode
}

void LidarRotationMotor::updateAngle()
{
  if (currentAngle_ == maxAngle_ || currentAngle_ == minAngle_)
  {
    LOG_ROS_INFO(this, "Change rotation direction");
    direction_ = !direction_;

    // Each minAngle_ means new sweep, increment counter for current scan
    if (currentAngle_ == minAngle_)
    {
      // Not all sweeps covered, increase counter for new sweep
      if (currentScanSweep_ < sweepsPerScan_)
      {
        currentScanSweep_++;
        LOG_ROS_INFO(this, "Sweep number: %d", currentScanSweep_);
      }
      // All sweeps covered, stop scan
      else
      {
        LOG_ROS_INFO(this, "All sweeps covered, stop scan");
        inScan_ = false;
        currentScanSweep_ = 0;

        float idleAngle = minAngle_ + (maxAngle_ - minAngle_) / 2;
        uint32_t pwm = getPWMValueFromAngle(idleAngle);
        moveMotor(pwm);
        if (endOfScanRequest() != RESULT_OK)
        {
          assert(0);
        }
        LOG_ROS_INFO(this, "Scan stopped");
      }
    }
  }

  // Update angle and direction
  currentAngle_ = direction_ ?  currentAngle_-angleIncrement_ : currentAngle_+angleIncrement_;

  if (!motorFakeMode_)
  {
    uint32_t pwm = getPWMValueFromAngle(currentAngle_);
    moveMotor(pwm);

    // Get the servo PWM value for that angle and move the motor to that position
    //uint32_t pwm = getPWMValueFromAngle(currentAngle_);
    //moveMotor(pwm);

  }
}

Result LidarRotationMotor::resetAngle()
{
  // @todo: move the motor to initial angle
  if (!motorFakeMode_)
  {
    uint32_t pwm = getPWMValueFromAngle(minAngle_);
    LOG_ROS_INFO(this, "Initializing motor. Set initial angle");
    moveMotor(pwm);
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  currentAngle_ = minAngle_;
  return Result::RESULT_OK;
}

Result LidarRotationMotor::initializeMotor()
{
  Result res;

  pwmController_ = new PCA9685(I2C_BUS, PCA9685_ADDRESS);
  if (pwmController_ == nullptr)
  {
    LOG_ROS_ERROR(this, "Failed to create pwm controller");
    return Result::RESULT_ERROR;
  }
  pwmController_->setPWMFreq(pwmFrequency_);  

  if (calibrationMode_)
  {
    //Result res = pwmCalibration();
  };

  res = resetAngle();

  if (res == Result::RESULT_OK)
  {
    motorState_ = MotorState::INITIALIZED;
  }

  return res;
}

Result LidarRotationMotor::pwmCalibration()
{
  uint32_t step = 10;

  LOG_ROS_INFO(this, "Press any key when you see motor movement to stop calibration.");

  for (uint32_t i = MIN_PWM_VALUE; i <= MAX_PWM_VALUE; i+=step)
  {
    moveMotor(i);
    LOG_ROS_INFO(this, "Current PWM: %d", i);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  LOG_ROS_INFO(this, "PWM Calibration finished");
  return Result::RESULT_OK;
}

uint32_t LidarRotationMotor::getPWMValueFromAngle(float angle)
{
  uint32_t pwm = 0;

  uint32_t pwmRange = (maxPwm_ - minPwm_);
  float angleRange = maxAngle_ - minAngle_;

  pwm = static_cast<uint32_t>(((angle - minAngle_) / angleRange) * pwmRange + minPwm_);

  return pwm;
}

Result LidarRotationMotor::moveMotor(uint32_t pwm)
{
  // @todo -> implement extra verbose traces by launch param
  if (false)
  {
    LOG_ROS_INFO(this, "set PWM to %d", pwm);
  }
  pwmController_->setPWM(1, pwm);
  return Result::RESULT_OK;
}

Result LidarRotationMotor::stopMotor()
{
  pwmController_->setPWMFreq(pwmFrequency_); // -> min pos
  //pwmController_->setPWMFreq(449); -> això fa a anar a la minima pos
  //pwmController_->setPWMFreq(100 o 300); -> això fa a anar a la maxima pos. per que?
  LOG_ROS_INFO(this, "set PWM to %d", minPwm_);
  pwmController_->setPWM(1, minPwm_);
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  pwmController_->setPWM(1, 0);
  return Result::RESULT_OK;
}

ServoMotorRange LidarRotationMotor::getPwmRange()
{
  // Calculate PWM values for minimum and maximum pulse widths
  int min_value = std::round(pwmFrequency_ * MIN_PULSE_WIDTH * RESOLUTION_PCA9685);
  int max_value = std::round(pwmFrequency_ * MAX_PULSE_WIDTH * RESOLUTION_PCA9685);

  // Ensure values are within the valid range (0-4095)
  min_value = std::max(0, std::min(RESOLUTION_PCA9685 - 1, min_value));
  max_value = std::max(0, std::min(RESOLUTION_PCA9685 - 1, max_value));

  LOG_ROS_INFO(this, "PWM range: %d - %d", min_value, max_value);

  return {min_value, max_value};
}

Result LidarRotationMotor::endOfScanRequest()
{
  Result res = RESULT_OK;

  LOG_ROS_INFO(this, "Requesting end of scan");

  // Wait for the service to be available
  if (!stopScanServiceClient_->wait_for_service(std::chrono::seconds(5))) 
  {
    LOG_ROS_ERROR(this, "Stop scan service not available after waiting");
    return res;
  }

  auto request = std::make_shared<lidar_pointcloud_scan::srv::StopScan::Request>();
  request->stop_reason = static_cast<int8_t>(EndScanReason::END);

  LOG_ROS_INFO(this, "Sending stop scan request.");
  auto future = stopScanServiceClient_->async_send_request(
    request, 
    std::bind(&LidarRotationMotor::endOfScanRequestCallback, this, std::placeholders::_1)
  );

  return res;
}

void LidarRotationMotor::endOfScanRequestCallback(rclcpp::Client<lidar_pointcloud_scan::srv::StopScan>::SharedFuture stopScanServiceFuture)
{
  auto response = stopScanServiceFuture.get();
  if (response->success)
  {
    LOG_ROS_INFO(this, "Stop scan service request successful");
  }
  else
  {
    LOG_ROS_ERROR(this, "Stop scan service request failed");
    assert(0);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRotationMotor>());
  rclcpp::shutdown();
  return 0;
}
