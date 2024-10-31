#include <memory>
#include <memory>
#include <thread>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp_action/client_goal_handle.hpp>
#include "action_msgs/srv/cancel_goal.hpp"
#include "lidar_pointcloud_scan/srv/start_scan.hpp"
#include "lidar_pointcloud_scan/srv/stop_scan.hpp"
#include "lidar_pointcloud_scan/action/motor_scan.hpp"
#include "lidar_pointcloud_scan/types.h"
#include "sensor_msgs/msg/joy.hpp"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*M_PI/180.)

constexpr double AXIS_THRESHOLD = 0.5;

using StartScanService = lidar_pointcloud_scan::srv::StartScan;
using StopScanService = lidar_pointcloud_scan::srv::StopScan;

using MotorScan = lidar_pointcloud_scan::action::MotorScan;
using GoalHandleMotorScan = rclcpp_action::ClientGoalHandle<MotorScan>;

class DeviceOrchestrator : public rclcpp::Node
{
public:
  DeviceOrchestrator();

// Private methods
private:
    /*
    * Get parameters for the node
    */
    void initParameters();

    /*
    * Initialize the scanning process.
    */
    void initScan ();

    /*
    * Stop the scanning process.
    */
    void stopScan (EndScanReason endReason);

    // Point Cloud Transformer Services
    Result initScanTransformer();
    void initScanTransformerCallback (rclcpp::Client<StartScanService>::SharedFuture startScanServiceFuture);
    Result stopScanTransformer(EndScanReason endReason);
    void stopScanTransformerCallback (rclcpp::Client<StopScanService>::SharedFuture stopScanServiceFuture);

    // Rotation Motor Scan Action
    void startMotorScan();
    void stopMotorScanCallback(const std::shared_ptr<action_msgs::srv::CancelGoal_Response> response);
    void motorScanGoalResponse(const GoalHandleMotorScan::SharedPtr & goalHandle);
    void motorScanFeedbackCallback(GoalHandleMotorScan::SharedPtr goalHandle, const std::shared_ptr<const MotorScan::Feedback> feedback);
    void motorScanResultCallback(const GoalHandleMotorScan::WrappedResult & result);

    // Keyboard/Joy control
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    JoyButton getPressedButton (const sensor_msgs::msg::Joy::SharedPtr msg);

// Private attributes
private:
    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber_;

    // Action Clients
    rclcpp_action::Client<MotorScan>::SharedPtr motorScanClient_;
    rclcpp_action::ClientGoalHandle<MotorScan>::SharedPtr motorScanGoalHandle_;

    // Service Clients
    rclcpp::Client<StartScanService>::SharedPtr startScanServiceTransformer_;
    rclcpp::Client<StopScanService>::SharedPtr stopScanServiceTransformer_;

    // Scan state attribute
    bool inScan_ = false;

    // Keep track of the goal request to avoid multiple requests
    bool motorScanGoalRequested_ = false;
};