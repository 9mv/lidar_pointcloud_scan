#include "lidar_pointcloud_scan/Motors.h"
#include "geometry_msgs/msg/point32.hpp"
#include <array>
#include <thread>

using JoyCoordinates = geometry_msgs::msg::Point32;

constexpr int NUM_MOTORS = 4;

constexpr float CHANGE_THRESHOLD = 0.1f;

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
    }

	~RobotController()
    {}

    Result move(const JoyCoordinates& axisInformation)
    {
        RobotMotorSpeeds robotSpeeds = getMotorParamsFromJoy(axisInformation);

        LOG_ROS_INFO(this, "Turning left wheels to speed %f and right wheels to speed %f", robotSpeeds.left, robotSpeeds.right);

        if (simulatedMode_)
        {
            return RESULT_OK;
        }

        // @todo -> real movements

        return RESULT_OK;
    }

    Result stop()
    {
        LOG_ROS_INFO(this, "Stop command");
        if (simulatedMode_)
        {
            return RESULT_OK;
        }
        motorFL_->stopMotor();
        motorFR_->stopMotor();
        motorRL_->stopMotor();
        motorRR_->stopMotor();
        return RESULT_OK;
    }

    Result initializeMotors()
    {
        LOG_ROS_INFO(this, "Initializing motors");
        motorFL_->initializeMotor();
        rclcpp::sleep_for(std::chrono::milliseconds(2000));
        motorFR_->initializeMotor();
        rclcpp::sleep_for(std::chrono::milliseconds(2000));
        motorRL_->initializeMotor();
        rclcpp::sleep_for(std::chrono::milliseconds(2000));
        motorRR_->initializeMotor();
        //std::thread t1FL(&DcMotor::initializeMotor, motorFL_);
        //std::thread t1FR(&DcMotor::initializeMotor, motorFR_);
        //std::thread t1RL(&DcMotor::initializeMotor, motorRL_);
        //std::thread t1RR(&DcMotor::initializeMotor, motorRR_);

        //t1FL.join();
        //t1FR.join();
        //t1RL.join();
        //t1RR.join();
        return RESULT_OK;
    }

    void setSimulatedMode(bool simulated)
    {
        simulatedMode_ = simulated;
    }

    rclcpp::Logger get_logger() const { return nodeLogger_; }
    const char* get_name() const { return nodeName_.c_str(); }

private:
    RobotMotorSpeeds getMotorParamsFromJoy(const JoyCoordinates& axis)
    {
        // @todo -> limit speed in robot_control component

        /*
        x = -1 y = 0 -> left (circles)
        x = 1 y = 0 -> right (circles)
        x = 0 y = -1 -> reverse
        x = 0 y = 1 -> forward
        x = 1 y = 1 -> right turn
        x = -1 y = 1 -> left turn
        x = 1 y = -1 -> left turn reverse
        x = -1 y = -1 -> right turn reverse
        */

        // Right now it's not expected as with keyboard only possible values
        // are -1, 0 and 1 (see device_orchestrator.cpp), but for the future, 
        // we might not want to apply small axis changes.
        float x = (std::abs(axis.x - currentMovement_.x) > CHANGE_THRESHOLD) ? axis.x : currentMovement_.x;
        float y = (std::abs(axis.y - currentMovement_.y) > CHANGE_THRESHOLD) ? axis.y : currentMovement_.y;

        currentMovement_.x = x;
        currentMovement_.y = y;

        // Assign speed to each motor
        RobotMotorSpeeds robotSpeeds;
        robotSpeeds.left = axis.y + axis.x/2.0;
        robotSpeeds.right = axis.y - axis.x/2.0;
        return robotSpeeds;
    }

private:
	DcMotor* motorFL_ = nullptr;
    DcMotor* motorFR_ = nullptr;
    DcMotor* motorRL_ = nullptr;
    DcMotor* motorRR_ = nullptr;

    bool simulatedMode_ = false;

    JoyCoordinates currentMovement_ = geometry_msgs::msg::Point32();

    rclcpp::Logger nodeLogger_;
    std::string nodeName_ = "RobotController";
};
