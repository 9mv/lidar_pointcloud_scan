#include "lidar_pointcloud_scan/lidar_rotation_motor.h"


LidarRotationMotor::LidarRotationMotor()
: Node("lidar_rotation_motor"), currentAngle_(-90)
{
  initParameters();

  publisher_ = this->create_publisher<lidar_pointcloud_scan::msg::Angle>("tilt_angle", 10); // https://docs.ros2.org/galactic/api/std_msgs/index-msg.html

  Result res = initializeMotor();
  if (res != Result::RESULT_OK)
  {
    LOG_ROS_ERROR(this, "failed to initialize motor");
    rclcpp::shutdown();
    return;
  }

  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(incrementPeriod_)), std::bind(&LidarRotationMotor::timer_callback, this));
}

void LidarRotationMotor::initParameters ()
{
  LOG_ROS_ERROR(this, "Initializing parameters");

  // Declare parameters
  this->declare_parameter<bool>("motorFakeMode", false);
  this->declare_parameter<float>("angleIncrement", 0.0);
  this->declare_parameter<float>("incrementPeriod", 0);

  // Get parameters
  this->get_parameter<bool>("motorFakeMode", motorFakeMode_);
  LOG_ROS_ERROR(this, "motorFakeMode: %s", motorFakeMode_ ? "true" : "false");

  this->get_parameter_or<float>("angleIncrement", angleIncrement_, 1.0);
  LOG_ROS_ERROR(this, "Motor is in fake mode: angleIncrement: %f", angleIncrement_);

  this->get_parameter_or<float>("incrementPeriod", incrementPeriod_, 1);
  LOG_ROS_ERROR(this, "incrementPeriod: %f ms", incrementPeriod_);
}

void LidarRotationMotor::timer_callback()
{
  auto message = lidar_pointcloud_scan::msg::Angle();
  message.angle = currentAngle_;
  updateAngle();
  publisher_->publish(message);

  //@todo -> this function should not manage the real motor behavior -> maintain only for fake mode
}

void LidarRotationMotor::updateAngle()
{
  // Update angle
  currentAngle_ = direction_ ? currentAngle_+angleIncrement_ : currentAngle_-angleIncrement_;
  if (currentAngle_ == MAX_ANGLE || currentAngle_ == MIN_ANGLE)
  {
    direction_ = !direction_;
  }

  /////////////////////////////////
  // CODI PER PROVES @todo -> delete
  if (currentAngle_ == MAX_ANGLE)
  {
    go = false;
  }
  //
  /////////////////////////////////


  if (!motorFakeMode_)
  {
    /////////////////////////////////
    // CODI PER PROVES @todo -> delete
    if (go) //@todo -> temporal to test. DELETE
    {
      uint32_t pwm = getPWMValueFromAngle(currentAngle_);
      moveMotor(pwm);
    }
    //
    /////////////////////////////////

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
    uint32_t pwm = getPWMValueFromAngle(-90);
    moveMotor(pwm);
    pwmController_->setPWMFreq(1000);
  }
  currentAngle_ = -90;
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

  ////////////////////////////////
  // CODI PER PROVES
  pwmController_->setPWMFreq(100);

  LOG_ROS_INFO(this, "SET 90 DEG POS");
  pwmController_->setPWM(1, 2000);
  rclcpp::sleep_for(std::chrono::seconds(2));

  LOG_ROS_INFO(this, "SET -90 DEG POS");
  pwmController_->setPWM(1, 0);
  rclcpp::sleep_for(std::chrono::seconds(2));
  //
  ///////////////////////////////

  LOG_ROS_INFO(this, "Initializing motor. Set to initial angle.");
  res = resetAngle();

  return res;
}

uint32_t LidarRotationMotor::getPWMValueFromAngle(float angle)
{
  uint32_t pwm = 0;

  uint32_t pwmRange = (MAX_PWM_VALUE - MIN_PWM_VALUE);
  uint32_t angleRange = (MAX_ANGLE - MIN_ANGLE);
  float finalAngle = std::max(MIN_ANGLE, std::min(angle, MAX_ANGLE));

  pwm = static_cast<uint32_t>(((angle - MIN_ANGLE) / angleRange) * pwmRange + MIN_PWM_VALUE);

  LOG_ROS_INFO(this, "PWM for angle (%f degrees) is %d", finalAngle, pwm);

  return pwm;
}

Result LidarRotationMotor::moveMotor(uint32_t pwm)
{
  LOG_ROS_INFO(this, "set PWM to %d", pwm);
  pwmController_->setPWM(1, pwm);
  return Result::RESULT_OK;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRotationMotor>());
  rclcpp::shutdown();
  return 0;
}
