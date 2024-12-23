#ifndef LIDAR_POINTCLOUD_SCAN_TYPES_H
#define LIDAR_POINTCLOUD_SCAN_TYPES_H
#include <cstdint>
// MACROS
#define LOG_ROS_ERROR(node, format, ...) RCLCPP_ERROR(node->get_logger(), "%s/%s(): " format, node->get_name(), __func__, ##__VA_ARGS__)
#define LOG_ROS_INFO(node, format, ...) RCLCPP_INFO(node->get_logger(), "%s/%s(): " format, node->get_name(), __func__, ##__VA_ARGS__)

// ENUMS
enum Result
{
    RESULT_ERROR,
    RESULT_OK
};

enum MotorState
{
    UNINITIALIZED,
    INITIALIZED
};

enum EndScanReason 
{
  CANCEL,
  END
};

enum ProcessingType 
{
  POINT_CLOUD_PROCESSING,
  LASER_SCAN_PROCESSING
};

enum JoyButtonPriority 
{
  PRIORITY_UNSET = -1,
  PRIORITY_SCAN = 0,
  PRIORITY_CANCEL = 1
};

enum JoyButton
{
  BUTTON_NONE = -1,
  BUTTON_SPACE = 0,
  BUTTON_SCAN = 1,
  BUTTON_CANCEL = 2
};

enum MotorDirection
{
  DIRECTION_FORWARD,
  DIRECTION_REVERSE
};

enum PointCloudTransformerState
{
  TRANSFORMER_NOT_READY,
  TRANSFORMER_READY,
  TRANSFORMER_WAIT_FOR_SCAN,
  TRANSFORMER_REQUEST_NEW_ANGLE
};

enum MotorIdentification
{
  MOTOR_FRONT_LEFT,
  MOTOR_FRONT_RIGHT,
  MOTOR_REAR_LEFT,
  MOTOR_REAR_RIGHT
};

// CONSTANTS
const std::unordered_map<JoyButton, JoyButtonPriority> BUTTON_PRIORITY_MAP = {
    {JoyButton::BUTTON_NONE, JoyButtonPriority::PRIORITY_UNSET},
    {JoyButton::BUTTON_SPACE, JoyButtonPriority::PRIORITY_UNSET},
    {JoyButton::BUTTON_SCAN, JoyButtonPriority::PRIORITY_SCAN},
    {JoyButton::BUTTON_CANCEL, JoyButtonPriority::PRIORITY_CANCEL}
};

// STRUCTS
struct PwmRange
{
    int min;
    int max;
};

struct PointCloudPoint {
  float x;
  float y;
  float z;
};

struct ServoMotorParams
{
  uint8_t channel = 0;
  float minAngle;
  float maxAngle;
};

struct DcMotorParams
{
  int forwardGpio;
  int reverseGpio;
  int speedPwmChannel;
  bool hasEncoder;
  int encoderGpio;
};

struct RobotMotorSpeeds
{
  float left;
  float right;
};

#endif // LIDAR_POINTCLOUD_SCAN_TYPES_H
