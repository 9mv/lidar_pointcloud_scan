#include <memory>
#include "PCA9685/PCA9685Manager.h"
#include "lidar_pointcloud_scan/types.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include <wiringPi.h>

// IMotor interface with some common motor functions
class IMotor {
public:
    IMotor(std::string nodeName, rclcpp::Logger rosLogger)
        : nodeName_(std::move(nodeName)), rosLogger_(std::move(rosLogger)) {}

    virtual ~IMotor() = default;

    virtual Result moveMotor(float pos) = 0;

    virtual Result stopMotor() = 0;

    // For logging purposes
    rclcpp::Logger get_logger() const { return rosLogger_; }
    const char* get_name() const { return nodeName_.c_str(); }
protected:
    std::string nodeName_;
    rclcpp::Logger rosLogger_;
};

class DcMotor : public IMotor
{
public:

    static constexpr double MAX_SPEED = 255.0;
    static constexpr int PCA9685_PWM_MAX = 4096;

    DcMotor(DcMotorParams motorParams, std::string motorInstanceName, rclcpp::Logger logger)
        : IMotor(motorInstanceName, logger), speedPwmChannel_(motorParams.speedPwmChannel), forwardGpio_(motorParams.forwardGpio), reverseGpio_(motorParams.reverseGpio)
    {
        pwmController_ = PCA9685Manager::getInstance();
        if (pwmController_ == nullptr)
        {
            LOG_ROS_ERROR(this, "Failed to get PWM controller");
            assert(0);
        }

        // Configure Raspberry GPIOs
        LOG_ROS_INFO(this, "Configuring GPIOs: forward %d, reverse %d and pwmChannel %d", forwardGpio_, reverseGpio_, motorParams.speedPwmChannel);
        wiringPiSetupGpio();
        pinMode(forwardGpio_, OUTPUT);
        pinMode(reverseGpio_, OUTPUT);
        if (motorParams.hasEncoder)
        {
            LOG_ROS_INFO(this, "Configuring GPIOs: encoder gpio %d", motorParams.encoderGpio);
            pinMode(motorParams.encoderGpio, INPUT);

            // @todo -> set encoder callbacks? Pending to see if the system will have encoders
            //pullUpDnControl(motorParams.encoderGpio, PUD_UP);
            //wiringPiISR(motorParams.encoderGpio, INT_EDGE_RISING, &DcMotor::encoderCallback);
            
        }

    }

    virtual ~DcMotor() override
    {
        stopMotor();
    }
    
    //void encoderCallback()
    //{

    //}
    

    Result moveMotor(MotorDirection direction, uint8_t speed)
    {
        // Get PWM for the desired speed: speed = 0 -> min Pwm / speed = 255 -> max Pwm
        uint32_t controllerSpeed = std::round(PCA9685_PWM_MAX * (speed / MAX_SPEED));

        LOG_ROS_INFO(this, "Moving motor: direction %d, speed %d, controllerSpeed %d", direction, speed, controllerSpeed);

        pwmController_->setPWM(speedPwmChannel_, controllerSpeed);

        digitalWrite(forwardGpio_, (direction == MotorDirection::DIRECTION_FORWARD) ? 1 : 0);
        digitalWrite(reverseGpio_, (direction == MotorDirection::DIRECTION_REVERSE) ? 1 : 0);

        return RESULT_OK;
    }

    Result moveMotor(float speed) override
    {
        (void)speed;
        return RESULT_OK;
    }

    Result stopMotor() override
    {
        digitalWrite(forwardGpio_, 0);
        digitalWrite(reverseGpio_, 0);
        pwmController_->setPWM(speedPwmChannel_, 0);
        return RESULT_OK;
    }

private:
    std::shared_ptr<PCA9685> pwmController_;

    int speedPwmChannel_;
    int forwardGpio_;
    int reverseGpio_;
};


class ServoMotor : public IMotor
{
public:
    static constexpr double MIN_PULSE_WIDTH = 0.0005; // 1ms, minimum servo pulse width
    static constexpr double MAX_PULSE_WIDTH = 0.0025; // 2ms, maximum servo pulse width

    ServoMotor(ServoMotorParams servoMotorParams, std::string nodeName, rclcpp::Logger logger)
        : IMotor(nodeName, logger)
    {
        pwmController_ = PCA9685Manager::getInstance();
        if (pwmController_ == nullptr)
        {
            LOG_ROS_ERROR(this, "Failed to create PWM controller");
            assert(0);
        }

        LOG_ROS_INFO(this, "ServoMotor constructor");

        if (servoMotorParams.channel != 0)
        {
            channel_ = servoMotorParams.channel;
        }

        LOG_ROS_INFO(this, "PWM Frequency %d", pwmController_->getPWMFreq());
        
        PwmRange range = getPwmRange();
        minPwm_ = range.min;
        maxPwm_ = range.max;

        minAngle_ = servoMotorParams.minAngle;
        maxAngle_ = servoMotorParams.maxAngle;
    }

    virtual ~ServoMotor() override
    {
        stopMotor();
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
        if (extendedROSLogging_)
        {
            LOG_ROS_INFO(this, "Set motor to angle %f, PWM: %d", angle, pwm);
        }
        pwmController_->setPWM(channel_, pwm);
        return RESULT_OK;
    }

    /*
    * Stop the rotation motor and pit it to initial angle
    * @return Result of the operation
    */
    Result stopMotor() override
    {
        pwmController_->setPWM(channel_, minPwm_);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        pwmController_->setPWM(channel_, 0);
        return RESULT_OK;
    }

    Result setPwm(int pwm)
    {
        pwmController_->setPWM(channel_, pwm);
        if (extendedROSLogging_)
        {
            LOG_ROS_INFO(this, "Set PWM to %d", minPwm_);
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        return RESULT_OK;
    }

    PwmRange getPwmRange()
    {
        // Calculate PWM values for minimum and maximum pulse widths
        int freq = pwmController_->getPWMFreq();
        LOG_ROS_INFO(this, "PWM Frequency %d", freq);
        int minValue = std::round(freq * MIN_PULSE_WIDTH * RESOLUTION_PCA9685);
        int maxValue = std::round(freq * MAX_PULSE_WIDTH * RESOLUTION_PCA9685);

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

private:
    // Servo driver board
    std::shared_ptr<PCA9685> pwmController_;

    uint32_t minPwm_ = 0;
    uint32_t maxPwm_ = 0;

    uint8_t channel_ = 0;

    float minAngle_ = 0;
    float maxAngle_ = 0;

    bool extendedROSLogging_ = false;
};
