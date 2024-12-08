
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lidar_pointcloud_scan/msg/angle.hpp"
#include "lidar_pointcloud_scan/srv/transformer_state.hpp"
#include "lidar_pointcloud_scan/action/motor_scan.hpp"

#include "lidar_pointcloud_scan/Motors.h"

using namespace std::chrono_literals;

using MotorScan = lidar_pointcloud_scan::action::MotorScan;
using GoalHandleMotorScan = rclcpp_action::ServerGoalHandle<MotorScan>;

using TransformerState = lidar_pointcloud_scan::srv::TransformerState;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

constexpr float MIN_ANGLE = -90.0;
constexpr float MAX_ANGLE = 90.0;
constexpr float ANGLE_RANGE = 180.0;

constexpr uint32_t MIN_PWM_VALUE = 0;
constexpr uint32_t MAX_PWM_VALUE = 4095;

constexpr uint8_t FEEDBACK_PROVIDER_LOOP_RATE = 10;
constexpr uint8_t SERVO_MOTOR_CHANNEL = 1;

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
    * MotorScan action server methods
    */
    Result initScanServer();
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MotorScan::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleMotorScan> goalHandle);
    void handleAccepted(const std::shared_ptr<GoalHandleMotorScan> goalHandle);
    void scanFeedBackProvider(const std::shared_ptr<GoalHandleMotorScan> goalHandle);
    
    /*
    * TransformerState service server methods
    */
    void handleTransformerState(const std::shared_ptr<TransformerState::Request> request,  std::shared_ptr<TransformerState::Response> response);

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
    * Actions to perform at the beginning of a scan
    */
    void startOfScanActions();

    /*
    * Actions to perform at the end of a scan
    * @param endReason: reason for the end of the scan
    */
    void endOfScanActions(EndScanReason endReason);

    /*
    * Reset the angle to 0
    * @return Result of the operation
    */
    Result resetAngle();


// Private attributes
private: 
    rclcpp::TimerBase::SharedPtr timer_;

    // Publishers
    rclcpp::Publisher<lidar_pointcloud_scan::msg::Angle>::SharedPtr anglePublisher_;

    // Services
    rclcpp::Service<TransformerState>::SharedPtr transformerStateService_;

    // Action Server
    rclcpp_action::Server<MotorScan>::SharedPtr motorScanActionServer_;
    
    // Flag to indicate if the scan is in progress
    bool inScan_ = false;

    // Current angle of the tilt motor
    float currentAngle_ = -90;

    // Direction of the rotation (true: clockwise, false: counter-clockwise)
    MotorDirection direction_ = DIRECTION_FORWARD;

    // Special motor modes
    bool motorFakeMode_ = false;
    bool calibrationMode_ = false;
    
    // Increment of the angle of the LiDAR tilt motor
    float angleIncrement_ = 1;

    // Angle increment period in ms
    float incrementPeriod_ = 250.0;

    // Low Level motor
    ServoMotor* motor_ = nullptr;

    // State of the motor
    MotorState motorState_ = UNINITIALIZED;

    // Corrections of motor rotation angle range
    float minAngle_ = MIN_ANGLE;
    float maxAngle_ = MAX_ANGLE;

    // Frequency of the PWM signal
    int pwmFrequency_ = 150;

    // Full sweeps to complete a scan (1 full sweep covers forward angles and backward angles)
    int sweepsPerScan_ = 1;

    // Attribute to keep track of the current scan sweep
    int currentScanSweep_ = 0;

    // Status of readiness of the PointCloudTransformer
    PointCloudTransformerState transformerState_ = TRANSFORMER_NOT_READY;
};
