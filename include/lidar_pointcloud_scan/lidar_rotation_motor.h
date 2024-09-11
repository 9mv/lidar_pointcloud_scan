
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lidar_pointcloud_scan/msg/angle.hpp"
#include "lidar_pointcloud_scan/types.h"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

const int MIN_ANGLE = -90.0;
const int MAX_ANGLE = 90.0;

class LidarRotationMotor : public rclcpp::Node
{
public:
    LidarRotationMotor();

//Private methods
private:
    /*
    * Get parameters for the node
    */
    void initParameters ();

    /*
    * Timer callback to publish the angle
    */
    void timer_callback();

    /*
    * Update the angle of the motor
    */
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

    // Publishers
    rclcpp::Publisher<lidar_pointcloud_scan::msg::Angle>::SharedPtr publisher_;
    
    // Current angle of the tilt motor
    float currentAngle_ = -90;

    // Direction of the rotation (true: clockwise, false: counter-clockwise)
    bool direction_ = true;

    // Attributes for fake mode
    bool motorFakeMode_ = false;
    float fakeAngleIncrement_ = 1;

    // Angle increment period in ms
    float incrementPeriod_ = 250.0;
};
