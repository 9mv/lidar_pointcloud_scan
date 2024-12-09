#include "lidar_pointcloud_scan/lidar_rotation_motor.h"


LidarRotationMotor::LidarRotationMotor()
: Node("lidar_rotation_motor"), currentAngle_(-90)
{
  initParameters();

  // Publishers
  anglePublisher_ = this->create_publisher<lidar_pointcloud_scan::msg::Angle>("tilt_angle", 10); // https://docs.ros2.org/galactic/api/std_msgs/index-msg.html

  // Services
  transformerStateService_ = this->create_service<TransformerState>(
    "transformer_state",
    std::bind(&LidarRotationMotor::handleTransformerState, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Actions
  initScanServer();

  Result res = initializeMotor();
  if (res != Result::RESULT_OK)
  {
    LOG_ROS_ERROR(this, "failed to initialize motor");
    rclcpp::shutdown();
    return;
  }

}

LidarRotationMotor::~LidarRotationMotor()
{
  if (!motorFakeMode_)
  {
    motor_->stopMotor();  
  }
  delete motor_;
}

void LidarRotationMotor::initParameters ()
{
  LOG_ROS_INFO(this, "Initializing parameters");

  // Declare parameters
  this->declare_parameter<bool>("motorFakeMode", false);
  this->declare_parameter<float>("angleIncrement", 0.0);
  this->declare_parameter<bool>("calibrationMode", false);
  this->declare_parameter<int>("pwmFrequency", 150);
  this->declare_parameter<float>("angleRange", ANGLE_RANGE);
  this->declare_parameter<int>("sweepsPerScan", 1);

  // Get parameters
  this->get_parameter<bool>("motorFakeMode", motorFakeMode_);
  LOG_ROS_INFO(this, "motorFakeMode: %s", motorFakeMode_ ? "true" : "false");

  this->get_parameter_or<float>("angleIncrement", angleIncrement_, 1.0);
  LOG_ROS_INFO(this, "angleIncrement: %f", angleIncrement_);

  this->get_parameter<bool>("calibrationMode", calibrationMode_);
  LOG_ROS_INFO(this, "calibrationMode: %s", calibrationMode_ ? "true" : "false");

  this->get_parameter_or<int>("pwmFrequency", pwmFrequency_, 150);

  this->get_parameter_or<int>("sweepsPerScan", sweepsPerScan_, 1);

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
  if (motorState_ == MotorState::UNINITIALIZED || inScan_)
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
  while (inScan_ && rclcpp::ok())
  {
    // Check if there is a cancel request
    if (goalHandle->is_canceling()) 
    {
      result->end_result = static_cast<int8_t>(EndScanReason::CANCEL);
      goalHandle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      endOfScanActions(EndScanReason::CANCEL);
      return;
    }

    // Update angle and move servo
    updateAngleActions();
  
    // Publish feedback
    // @todo -> check if some kind of useful feedback can be sent to the orchestrator
    feedback->angle = currentAngle_;
    goalHandle->publish_feedback(feedback);

    loopRate.sleep();
  }

  // Send the goal result after exiting thread loop
  if (rclcpp::ok()) 
  {
    result->end_result = static_cast<int8_t>(EndScanReason::END);
    goalHandle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
  inScan_ = false;
}

void LidarRotationMotor::handleTransformerState(const std::shared_ptr<TransformerState::Request> request,  std::shared_ptr<TransformerState::Response> response)
{
  transformerState_ = static_cast<PointCloudTransformerState>(request->state);

  //LOG_ROS_INFO(this, "New transformer state: %d", transformerState_);

  // @todo -> configure the response
  response->success = true;
}

void LidarRotationMotor::updateAngleActions()
{
  if (motorState_ == MotorState::UNINITIALIZED || !inScan_ 
      || (!motorFakeMode_ && (transformerState_ == PointCloudTransformerState::TRANSFORMER_NOT_READY || transformerState_ == PointCloudTransformerState::TRANSFORMER_WAIT_FOR_SCAN)))
  {
    return;
  }

  auto message = lidar_pointcloud_scan::msg::Angle();
  message.angle = currentAngle_;
  updateAngle();
  anglePublisher_->publish(message);

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
    motor_->moveMotor(currentAngle_);

    // Wait for the motor to move
    int expectedMoveTimeMs = static_cast<int>(std::ceil((ROTATION_SERVO_MIN_SPEED/60.0) * 1000.0 * angleIncrement_) * (1 + MOVE_WAIT_MARGIN));
    //LOG_ROS_INFO(this, "New angle: %f and expected wait time of %d ms", currentAngle_, expectedMoveTimeMs);
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(expectedMoveTimeMs)));
  }
}

void LidarRotationMotor::startOfScanActions()
{
  resetAngle();
  direction_ = DIRECTION_FORWARD;

  // Move the motor to initial angle
  if (!motorFakeMode_)
  {
    LOG_ROS_INFO(this, "Set initial angle");
    motor_->moveMotor(minAngle_);
    // Stabilization time
    rclcpp::sleep_for(std::chrono::seconds(1));
    LOG_ROS_INFO(this, "Motor set to initial angle");
  }
  currentAngle_ = minAngle_;

  // Let the movement end
  LOG_ROS_INFO(this, "reset to initial angle %f", currentAngle_);
  inScan_ = true;
}

void LidarRotationMotor::endOfScanActions(EndScanReason endReason)
{
    (void)endReason;
    float idleAngle = motor_->getMotorIdleAngle();
    if (!motorFakeMode_)
    {
      motor_->moveMotor(idleAngle);
    }
    inScan_ = false;
    currentScanSweep_ = 0;
    currentAngle_ = idleAngle;
    LOG_ROS_INFO(this, "set to idle angle %f", currentAngle_);
}

Result LidarRotationMotor::resetAngle()
{
  float idleAngle = motor_->getMotorIdleAngle();
  if (!motorFakeMode_)
  {
    LOG_ROS_INFO(this, "Initializing motor. Set idle angle %f", idleAngle);
    motor_->moveMotor(idleAngle);
    // Stabilization time
    rclcpp::sleep_for(std::chrono::seconds(1));
    LOG_ROS_INFO(this, "Motor reset");
  }
  currentAngle_ = idleAngle;
  return Result::RESULT_OK;
}

Result LidarRotationMotor::initializeMotor()
{
  Result res;

  ServoMotorParams params{SERVO_MOTOR_CHANNEL, minAngle_, maxAngle_};
  motor_ = new ServoMotor(params, this->get_name(), this->get_logger()); //@todo -> delete pwmFrequency_ from node params

  if (motor_ == nullptr)
  {
    LOG_ROS_ERROR(this, "Failed to create motor");
    return Result::RESULT_ERROR;
  }

  res = resetAngle();

  if (res == Result::RESULT_OK)
  {
    motorState_ = MotorState::INITIALIZED;
  }

  return res;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRotationMotor>());
  rclcpp::shutdown();
  return 0;
}
