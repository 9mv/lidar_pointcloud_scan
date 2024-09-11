#include "lidar_pointcloud_scan/lidar_rotation_motor.h"


LidarRotationMotor::LidarRotationMotor()
: Node("lidar_rotation_motor"), currentAngle_(0)
{
  publisher_ = this->create_publisher<lidar_pointcloud_scan::msg::Angle>("tilt_angle", 10); // https://docs.ros2.org/galactic/api/std_msgs/index-msg.html

  Result res = initializeMotor();
  if (res != Result::RESULT_OK)
  {
    LOG_ERROR(this, "failed to initialize motor");
    rclcpp::shutdown();
    return;
  }

  timer_ = this->create_wall_timer(250ms, std::bind(&LidarRotationMotor::timer_callback, this));
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
  currentAngle_ = direction_ ? currentAngle_+1 : currentAngle_-1;
  if (currentAngle_ == MAX_ANGLE || currentAngle_ == MIN_ANGLE)
  {
    direction_ = !direction_;
  }
}

Result LidarRotationMotor::resetAngle()
{
  // @todo: move the motor to angle 0
  return Result::RESULT_OK;
}

Result LidarRotationMotor::initializeMotor()
{
  Result res;
  LOG_INFO(this, "Initializing motor. Set to angle 0.");
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
