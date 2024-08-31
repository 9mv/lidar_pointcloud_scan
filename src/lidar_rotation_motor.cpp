
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "lidar_pointcloud_scan/msg/angle.hpp"
#include "lidar_pointcloud_scan/types.h"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180; //180;

class LidarRotationMotor : public rclcpp::Node
{
  public:
    LidarRotationMotor()
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

      timer_ = this->create_wall_timer(500ms, std::bind(&LidarRotationMotor::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = lidar_pointcloud_scan::msg::Angle();
      message.angle = currentAngle_;
      LOG_INFO(this, "publishing angle: '%d'", message.angle);
      publisher_->publish(message);
      updateAngle();
    }

    void updateAngle()
    {
      currentAngle_ = direction_ ? currentAngle_+1 : currentAngle_-1;
      if (currentAngle_ == MAX_ANGLE || currentAngle_ == MIN_ANGLE)
      {
        direction_ = !direction_;
      }
    }

    /*
    * Reset the angle to 0
    * @return Result of the operation
    */
    Result resetAngle()
    {
      // @todo: move the motor to angle 0
      return Result::RESULT_OK;
    }

    /*
    * Initializing motor actions
    * @return Result of the operation
    */
    Result initializeMotor()
    {
      Result res;
      LOG_INFO(this, "Initializing motor. Set to angle 0.");
      res = resetAngle();

      return res;
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<lidar_pointcloud_scan::msg::Angle>::SharedPtr publisher_;
    int currentAngle_ = 0;
    bool direction_ = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRotationMotor>());
  rclcpp::shutdown();
  return 0;
}
