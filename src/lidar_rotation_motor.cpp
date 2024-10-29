#include "lidar_pointcloud_scan/lidar_rotation_motor.h"


LidarRotationMotor::LidarRotationMotor()
: Node("lidar_rotation_motor"), currentAngle_(-90)
{
  initParameters();

  publisher_ = this->create_publisher<lidar_pointcloud_scan::msg::Angle>("tilt_angle", 10); // https://docs.ros2.org/galactic/api/std_msgs/index-msg.html

  initScanServer();

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

Result LidarRotationMotor::initScanServer()
{
  motorScanActionServer_ = rclcpp_action::create_server<MotorScan>(
      this,
      "motor_scan",
      std::bind(&LidarRotationMotor::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LidarRotationMotor::handleCancel, this, std::placeholders::_1),
      std::bind(&LidarRotationMotor::handleAccepted, this, std::placeholders::_1));
  
  return RESULT_OK;
}

rclcpp_action::GoalResponse LidarRotationMotor::handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MotorScan::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request UUID x");
  (void)uuid;
  (void)goal;
  if (motorState_ == MotorState::UNINITIALIZED || !inScan_)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  else
  {
    startOfScanActions();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

}

rclcpp_action::CancelResponse LidarRotationMotor::handleCancel(const std::shared_ptr<GoalHandleMotorScan> goalHandle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goalHandle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LidarRotationMotor::handleAccepted(const std::shared_ptr<GoalHandleMotorScan> goalHandle)
{
  using namespace std::placeholders;
  // Scan feedback executing in new thread to avoid blocking the execution of the node
  std::thread{std::bind(&LidarRotationMotor::scanFeedBackProvider, this, std::placeholders::_1), goalHandle}.detach();
}

// @todo -> I could implement the updateAngle based on each feedback + remove the looprate based on hardcoded value. Would it add much delay or operations?
void LidarRotationMotor::scanFeedBackProvider(const std::shared_ptr<GoalHandleMotorScan> goalHandle)
{
  rclcpp::Rate loopRate(FEEDBACK_PROVIDER_LOOP_RATE);
  const auto goal = goalHandle->get_goal();
  auto feedback = std::make_shared<MotorScan::Feedback>();
  auto result = std::make_shared<MotorScan::Result>();
  
  // Iterate at FEEDBACK_PROVIDER_LOOP_RATE Hz until motor has finished (inScan_ == false)
  while (inScan_ && rclcpp::ok()) {
    // Check if there is a cancel request
    if (goalHandle->is_canceling()) {
      result->end_result = static_cast<int8_t>(EndScanReason::CANCEL);
      goalHandle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      endOfScanActions(EndScanReason::CANCEL);
      return;
    }
    // Publish feedback
    // @todo -> check if some kind of useful feedback can be sent to the orchestrator
    feedback->angle = currentAngle_;
    goalHandle->publish_feedback(feedback);

    loopRate.sleep();
  }

  // Send the goal result after exiting thread loop
  if (rclcpp::ok()) {
    result->end_result = static_cast<int8_t>(EndScanReason::END);
    goalHandle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
  inScan_ = false;
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
//  LOG_ROS_INFO(this, "Current angle: %f and direction is %d", currentAngle_, direction_);
  if (currentAngle_ == maxAngle_ || currentAngle_ == minAngle_)
  {
    LOG_ROS_INFO(this, "Change rotation direction");

    // Each minAngle_ means new sweep, increment counter for current scan
    if (currentAngle_ == minAngle_)
    {
      direction_ = DIRECTION_FORWARD;
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
        endOfScanActions(EndScanReason::END);
        LOG_ROS_INFO(this, "Scan stopped");
        return;
      }
    }
    else
    {
      direction_ = DIRECTION_REVERSE;
    }
  }
  else if(currentAngle_ > maxAngle_ || currentAngle_ < minAngle_)
  {
    LOG_ROS_ERROR(this, "Angle out of range (%f). Shutdown...", currentAngle_);
    rclcpp::shutdown();
  }

  // Update angle and direction
  currentAngle_ = direction_==DIRECTION_FORWARD ? currentAngle_+angleIncrement_ : currentAngle_-angleIncrement_;

  if (!motorFakeMode_)
  {
    uint32_t pwm = getPWMValueFromAngle(currentAngle_);
    moveMotor(pwm);
  }
}

void LidarRotationMotor::startOfScanActions()
{
  resetAngle();
  direction_ = DIRECTION_FORWARD;
  LOG_ROS_INFO(this, "reset to initial angle %f", currentAngle_);
  inScan_ = true;
}

void LidarRotationMotor::endOfScanActions(EndScanReason endReason)
{
    (void)endReason;
    float idleAngle = minAngle_ + (maxAngle_ - minAngle_) / 2;
    uint32_t pwm = getPWMValueFromAngle(idleAngle);
    if (!motorFakeMode_)
    {
      moveMotor(pwm);
    }
    inScan_ = false;
    currentScanSweep_ = 0;
    currentAngle_ = idleAngle;
    LOG_ROS_INFO(this, "set to idle angle %f", idleAngle);
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
  // @todo -> delete this function if there is no real use
  /*
  uint32_t step = 10;

  LOG_ROS_INFO(this, "Press any key when you see motor movement to stop calibration.");

  for (uint32_t i = MIN_PWM_VALUE; i <= MAX_PWM_VALUE; i+=step)
  {
    moveMotor(i);
    LOG_ROS_INFO(this, "Current PWM: %d", i);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  LOG_ROS_INFO(this, "PWM Calibration finished");
  */
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
  // @todo -> implement extra verbose traces by launch param. false meanwhile.
  if (false)
  {
    LOG_ROS_INFO(this, "set PWM to %d", pwm);
  }
  pwmController_->setPWM(1, pwm);
  return Result::RESULT_OK;
}

Result LidarRotationMotor::stopMotor()
{
  pwmController_->setPWMFreq(pwmFrequency_);
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRotationMotor>());
  rclcpp::shutdown();
  return 0;
}
