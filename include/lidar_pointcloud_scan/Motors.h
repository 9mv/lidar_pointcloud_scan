#include <memory>
#include "PCA9685/PCA9685.h"
#include "lidar_pointcloud_scan/types.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

// IMotor interface with some common motor functions
class IMotor {
public:
    IMotor(std::string nodeName, rclcpp::Logger rosLogger)
        : nodeName_(std::move(nodeName)), rosLogger_(std::move(rosLogger)) {}

    virtual ~IMotor() = default;

    virtual Result moveMotor(float pos) = 0;

    virtual Result stopMotor() = 0;
protected:
    std::string nodeName_;
    rclcpp::Logger rosLogger_;
};

class ServoMotor : public IMotor
{
public:
    static constexpr double MIN_PULSE_WIDTH = 0.0005; // 1ms, minimum servo pulse width
    static constexpr double MAX_PULSE_WIDTH = 0.0025; // 2ms, maximum servo pulse width

    static constexpr int I2C_BUS = 1;
    static constexpr int PCA9685_ADDRESS = 0x40;

    ServoMotor(int pwmFrequency, ServoMotorConstants servoMotorConstants, std::string nodeName, rclcpp::Logger logger)
        : IMotor(nodeName, logger), pwmFrequency_(pwmFrequency)
    {
        pwmController_ = new PCA9685(1, 0x40);
        if (pwmController_ == nullptr)
        {
            LOG_ROS_ERROR(this, "Failed to create PWM controller");
            assert(0);
        }

        LOG_ROS_INFO(this, "PWM Frequency: %d", pwmFrequency_);
        LOG_ROS_INFO(this, "PCA9685 Address: %d", PCA9685_ADDRESS);

        ServoMotorRange range = getPwmRange();
        minPwm_ = range.min;
        maxPwm_ = range.max;

        minAngle_ = servoMotorConstants.minAngle;
        maxAngle_ = servoMotorConstants.maxAngle;
    }

    virtual ~ServoMotor() override
    {
        stopMotor();
        delete pwmController_;
    }

    /*
    * Move the rotation motor
    * @param angle: angle to move the motor to
    * @return Result of the operation
    */
    Result moveMotor(float angle) override
    {
        int pwm = getPWMValueFromAngle(angle);

        // @todo -> implement extra verbose traces by launch param. false meanwhile.
        //if (extendedROSLogging_)
        //{
            //LOG_ROS_INFO(this, "Set PWM to %d", pwm);
        //}
        pwmController_->setPWM(1, pwm);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        int pwm_now = pwmController_->getPWM(1);
        LOG_ROS_INFO(this, "angle %f, pwm_now: %d", angle, pwm_now);
        LOG_ROS_INFO(this, "pwmFrequency: %d", pwmFrequency_);
        return RESULT_OK;
    }

    /*
    * Stop the rotation motor and pit it to initial angle
    * @return Result of the operation
    */
    Result stopMotor() override
    {
        pwmController_->setPWM(1, minPwm_);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        pwmController_->setPWM(1, 0);
        return RESULT_OK;
    }

    Result setPwm(int pwm)
    {
        pwmController_->setPWM(1, pwm);
        if (extendedROSLogging_)
        {
            LOG_ROS_INFO(this, "Set PWM to %d", minPwm_);
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        return RESULT_OK;
    }

    ServoMotorRange getPwmRange()
    {
        // Calculate PWM values for minimum and maximum pulse widths
        int minValue = std::round(pwmFrequency_ * MIN_PULSE_WIDTH * RESOLUTION_PCA9685);
        int maxValue = std::round(pwmFrequency_ * MAX_PULSE_WIDTH * RESOLUTION_PCA9685);

        // Ensure values are within the valid range (0-4095)
        minValue = std::max(0, std::min(RESOLUTION_PCA9685 - 1, minValue));
        maxValue = std::max(0, std::min(RESOLUTION_PCA9685 - 1, maxValue));

        LOG_ROS_INFO(this, "PWM range: %d - %d", minValue, maxValue);

        return {minValue, maxValue};
    }

    /*
    * Get the PWM to turn motor to the given angle
    * @param angle: Angle to turn the motor to
    * @return uint32_t: PWM value 
    */
    uint32_t getPWMValueFromAngle(float angle)
    {
        uint32_t pwm = 0;

        uint32_t pwmRange = (maxPwm_ - minPwm_);
        float angleRange = maxAngle_ - minAngle_;

        pwm = static_cast<uint32_t>(((angle - minAngle_) / angleRange) * pwmRange + minPwm_);

        if (extendedROSLogging_)
        {
            LOG_ROS_INFO(this, "angle: %f, pwm: %d", angle, pwm);
        }
        return pwm;
    }

    void setPwmFreq(int frequency)
    {
        LOG_ROS_INFO(this, "Set PWM Frequency to %d", frequency);
        pwmFrequency_ = frequency;
        
        ServoMotorRange range = getPwmRange();
        minPwm_ = range.min;
        maxPwm_ = range.max;
    }

    /*
    * Get the idle angle of the motor
    * @return float: idle angle
    * @todo -> implement this as a real idle angle and not the mid angle
    */
    float getMotorIdleAngle()
    {
        return (minAngle_ + (maxAngle_ - minAngle_) / 2);
    }

    /*
    * Get the pwm corresponding to the idle angle of the motor
    * @return uint32_t: pwm of the motor to achieve idle angle
    */
    uint32_t getMotorIdlePwm()
    {
        return getPWMValueFromAngle(getMotorIdleAngle());
    }

    // For logging purposes
    rclcpp::Logger get_logger() const { return rosLogger_; }
    const char* get_name() const { return nodeName_.c_str(); }


private:
    PCA9685* pwmController_;
    uint32_t pwmFrequency_;

    uint32_t minPwm_ = 0;
    uint32_t maxPwm_ = 0;

    float minAngle_ = 0;
    float maxAngle_ = 0;

    bool extendedROSLogging_ = true;
};
