// MACROS
#define LOG_ROS_ERROR(node, format, ...) RCLCPP_ERROR(node->get_logger(), "%s/%s(): " format, node->get_name(), __func__, ##__VA_ARGS__)
#define LOG_ROS_INFO(node, format, ...) RCLCPP_INFO(node->get_logger(), "%s/%s(): " format, node->get_name(), __func__, ##__VA_ARGS__)

// ENUMS
enum Result
{
    RESULT_ERROR,
    RESULT_OK
};