#include "lidar_pointcloud_scan/lidar_rotation_motor.h"


LidarRotationMotor::LidarRotationMotor()
: Node("lidar_rotation_motor"), currentAngle_(-90)
{
  initParameters();

  publisher_ = this->create_publisher<lidar_pointcloud_scan::msg::Angle>("tilt_angle", 10); // https://docs.ros2.org/galactic/api/std_msgs/index-msg.html

  Result res = initializeMotor();
  if (res != Result::RESULT_OK)
  {
    LOG_ERROR(this, "failed to initialize motor");
    rclcpp::shutdown();
    return;
  }

  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(incrementPeriod_)), std::bind(&LidarRotationMotor::timer_callback, this));
}

void LidarRotationMotor::initParameters ()
{
  LOG_INFO(this, "Initializing parameters");

  // Declare parameters
  this->declare_parameter<bool>("motorFakeMode", false);
  this->declare_parameter<float>("fakeAngleIncrement", 0.0);
  this->declare_parameter<float>("incrementPeriod", 0);

  // Get parameters
  this->get_parameter<bool>("motorFakeMode", motorFakeMode_);
  LOG_INFO(this, "motorFakeMode: %s", motorFakeMode_ ? "true" : "false");

  if (motorFakeMode_)
  {
    this->get_parameter_or<float>("fakeAngleIncrement", fakeAngleIncrement_, 1.0);
    LOG_INFO(this, "Motor is in fake mode: fakeAngleIncrement: %f", fakeAngleIncrement_);
  }

  this->get_parameter_or<float>("incrementPeriod", incrementPeriod_, 1);
  LOG_INFO(this, "incrementPeriod: %f ms", incrementPeriod_);
}

void LidarRotationMotor::timer_callback()
{
  auto message = lidar_pointcloud_scan::msg::Angle();
  message.angle = currentAngle_;
  //LOG_INFO(this, "publishing angle: '%d'", message.angle);
  publisher_->publish(message);
  updateAngle();
}

void LidarRotationMotor::updateAngle()
{
  if (motorFakeMode_)
  {
    // @todo: move the motor to the new angle
    currentAngle_ = direction_ ? currentAngle_+fakeAngleIncrement_ : currentAngle_-fakeAngleIncrement_;
    if (currentAngle_ == MAX_ANGLE || currentAngle_ == MIN_ANGLE)
    {
      direction_ = !direction_;
    }
  }
  else
  {
    // @todo: real motor movement here
  }
}

Result LidarRotationMotor::resetAngle()
{
  // @todo: move the motor to initial angle
  if (motorFakeMode_)
  {
    currentAngle_ = -90;
  }
  return Result::RESULT_OK;
}

Result LidarRotationMotor::initializeMotor()
{
  Result res;
  LOG_INFO(this, "Initializing motor. Set to initial angle.");
  res = resetAngle();

  return res;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRotationMotor>());
  rclcpp::shutdown();
  return 0;
}
