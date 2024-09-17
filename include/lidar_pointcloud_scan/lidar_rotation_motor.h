
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lidar_pointcloud_scan/msg/angle.hpp"
#include "lidar_pointcloud_scan/types.h"

#include "PCA9685/PCA9685.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

constexpr float MIN_ANGLE = -90.0;
constexpr float MAX_ANGLE = 90.0;

// 2048 is angle 0º
constexpr uint32_t MIN_PWM_VALUE = 0;
constexpr uint32_t MAX_PWM_VALUE = 4095;

constexpr int I2C_BUS = 1;
constexpr int PCA9685_ADDRESS = 0x40;


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
    * Initializing motor actions
    * @return Result of the operation
    */
    Result initializeMotor();

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
    * Get the PWM to turn motor to the given angle
    * @return uint32_t PWM value 
    */
    uint32_t getPWMValueFromAngle(float angle);

    /*
    * Move the rotation motor
    * @return Result of the operation
    */
    Result moveMotor(uint32_t pwm);

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
    
    // Increment of the angle of the LiDAR tilt motor
    float angleIncrement_ = 1;

    // Angle increment period in ms
    float incrementPeriod_ = 250.0;

    PCA9685* pwmController_ = nullptr;

    //@todo -> temporal to test. DELETE
    bool go = true;
};
