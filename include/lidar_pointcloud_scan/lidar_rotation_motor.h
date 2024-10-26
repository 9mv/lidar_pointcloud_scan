
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lidar_pointcloud_scan/msg/angle.hpp"
#include "lidar_pointcloud_scan/srv/stop_scan.hpp"
#include "lidar_pointcloud_scan/types.h"

#include "PCA9685/PCA9685.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

constexpr float MIN_ANGLE = -90.0;
constexpr float MAX_ANGLE = 90.0;
constexpr float ANGLE_RANGE = 180.0;

constexpr uint32_t MIN_PWM_VALUE = 0;
constexpr uint32_t MAX_PWM_VALUE = 4095;

constexpr int I2C_BUS = 1;
constexpr int PCA9685_ADDRESS = 0x40;

constexpr double MIN_PULSE_WIDTH = 0.0005; // 1ms, minimum servo pulse width
constexpr double MAX_PULSE_WIDTH = 0.0025; // 2ms, maximum servo pulse width

class LidarRotationMotor : public rclcpp::Node
{
public:
    LidarRotationMotor();
    ~LidarRotationMotor();

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
    * Tilt motor calibration routine
    * @return Result of the operation
    */
    Result pwmCalibration();

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

    /*
    * Stop the rotation motor and pit it to initial angle
    * @return Result of the operation
    */
    Result stopMotor();

    /*
    * Get the PWM operation range of the motor
    * @return ServoMotorRange object with min and max PWM values
    */
    ServoMotorRange getPwmRange();

    /*
    * Request the end of a scan
    * @return Result of the operation
    */
    Result endOfScanRequest();

    /*
    * Callback for the end of scan service
    * @param stopScanServiceFuture Future of the stop scan service to check the result
    */
    void endOfScanRequestCallback(rclcpp::Client<lidar_pointcloud_scan::srv::StopScan>::SharedFuture stopScanServiceFuture);

// Private attributes
private: 
    rclcpp::TimerBase::SharedPtr timer_;

    // Publishers
    rclcpp::Publisher<lidar_pointcloud_scan::msg::Angle>::SharedPtr publisher_;

    // Service Clients
    rclcpp::Client<lidar_pointcloud_scan::srv::StopScan>::SharedPtr stopScanServiceClient_;
    
    // Flag to indicate if the scan is in progress
    bool inScan_ = true;

    // Current angle of the tilt motor
    float currentAngle_ = -90;

    // Direction of the rotation (true: clockwise, false: counter-clockwise)
    bool direction_ = true;

    // Special motor modes
    bool motorFakeMode_ = false;
    bool calibrationMode_ = false;
    
    // Increment of the angle of the LiDAR tilt motor
    float angleIncrement_ = 1;

    // Angle increment period in ms
    float incrementPeriod_ = 250.0;

    // Servo driver board
    PCA9685* pwmController_ = nullptr;

    //@todo -> temporal to test. DELETE
    bool go = true;

    // State of the motor
    MotorState motorState_ = UNINITIALIZED;

    // Corrections of PWM range
    int minPwm_ = MIN_PWM_VALUE;
    int maxPwm_ = MAX_PWM_VALUE;

    // Corrections of motor rotation angle range
    float minAngle_ = MIN_ANGLE;
    float maxAngle_ = MAX_ANGLE;

    // Frequency of the PWM signal
    int pwmFrequency_ = 150;

    // Full sweeps to complete a scan (1 full sweep covers forward angles and backward angles)
    int sweepsPerScan_ = 1;

    // Attribute to keep track of the current scan sweep
    int currentScanSweep_ = 0;
};
