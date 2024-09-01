
#include <chrono>
#include <functional>
#include <memory>

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
    LidarRotationMotor();

//Private methods
private:
    void timer_callback();

    void updateAngle();

    /*
    * Reset the angle to 0
    * @return Result of the operation
    */
    Result resetAngle();

    /*
    * Initializing motor actions
    * @return Result of the operation
    */
    Result initializeMotor();

// Private attributes
private: 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<lidar_pointcloud_scan::msg::Angle>::SharedPtr publisher_;
    int currentAngle_ = 0;
    bool direction_ = true;
};
