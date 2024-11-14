#include "lidar_pointcloud_scan/Motors.h"
#include "geometry_msgs/msg/point32.hpp"
#include <array>
#include <thread>

using JoyCoordinates = geometry_msgs::msg::Point32;

constexpr int NUM_MOTORS = 4;

constexpr std::array<DcMotorParams, NUM_MOTORS> motorConfigs = 
{{
    //  FORWARD GPIO        REVERSE GPIO    PWM CHANNEL     HAS_ENCODER/ENCODER GPIO (-1 if not, for clarity)
    {17,                    27,             2,              false, 24},    // FRONT_LEFT
    {6,                     13,             3,              false, 23},    // FRONT_RIGHT
    {21,                    20,             4,              false, 26},    // BACK_LEFT
    {10,                    9,              5,              false, 16}     // BACK_RIGHT
}};

class RobotController
{
public:
	RobotController(rclcpp::Logger nodeLogger) : nodeLogger_(nodeLogger)
    {
        motorFL_ = new DcMotor(motorConfigs[MOTOR_FRONT_LEFT], "WheelMotorFrontLeft", nodeLogger_);
        motorFR_ = new DcMotor(motorConfigs[MOTOR_FRONT_RIGHT], "WheelMotorFrontRight", nodeLogger_);
        motorRL_ = new DcMotor(motorConfigs[MOTOR_REAR_LEFT], "WheelMotorRearLeft", nodeLogger_);
        motorRR_ = new DcMotor(motorConfigs[MOTOR_REAR_RIGHT], "WheelMotorRearRight", nodeLogger_);

        initializeMotors();
    }

	~RobotController()
    {}

    Result move(const JoyCoordinates& axisInformation)
    {
        getMotorParamsFromJoy(axisInformation);
        return RESULT_OK;
    }

    // TEMPORAL TEST CODE TO TEST MOTOR
    Result move()
    {
        LOG_ROS_INFO(this,"[TEST] Move command");
        motorFL_->moveMotor(MotorDirection::DIRECTION_FORWARD, 100);
        return RESULT_OK;
    }
    // END TEST CODE

    Result stop()
    {
        // TEMPORAL TEST CODE TO TEST MOTOR
        LOG_ROS_INFO(this, "[TEST] Stop command");
        motorFL_->stopMotor();
        return RESULT_OK;
        // END TEST CODE
    }

    Result initializeMotors()
    {
        LOG_ROS_INFO(this, "Initializing motors");
        std::thread t1FL(&DcMotor::initializeMotor, motorFL_);
        //std::thread t1FR(&DcMotor::initializeMotor, motorFR_);
        //std::thread t1RL(&DcMotor::initializeMotor, motorRL_);
        //std::thread t1RR(&DcMotor::initializeMotor, motorRR_);

        t1FL.join();
        //t1FR.join();
        //t1RL.join();
        //t1RR.join();
        return RESULT_OK;
    }

    rclcpp::Logger get_logger() const { return nodeLogger_; }
    const char* get_name() const { return nodeName_.c_str(); }

private:
    void getMotorParamsFromJoy(const JoyCoordinates& axis)
    {
        (void)axis;
        // @todo -> limit speed in robot_control component

        // Assign speed to each motor
    }

private:
	DcMotor* motorFL_ = nullptr;
    DcMotor* motorFR_ = nullptr;
    DcMotor* motorRL_ = nullptr;
    DcMotor* motorRR_ = nullptr;

    rclcpp::Logger nodeLogger_;
    std::string nodeName_ = "RobotController";
};

/*
EN1.A -> PWM PCA9685 CH2 (1)
EN1.B -> PWM PCA9685 CH3 (2)
EN2.A -> PWM PCA9685 CH4 (3)
EN2.B -> PWM PCA9685 CH5 (4)

IN1.1 -> GPIO17
IN1.2 -> GPIO27
IN1.3 -> GPIO6
IN1.4 -> GPIO13
IN2.1 -> GPIO21
IN2.2 -> GPIO20
IN2.3 -> GPIO10
IN2.4 -> GPIO9

ENC1.A -> GPIO24
ENC1.B -> GPIO23
ENC2.A -> GPIO26
ENC2.B -> GPIO16
*/