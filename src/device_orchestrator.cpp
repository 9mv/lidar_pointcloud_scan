#include "lidar_pointcloud_scan/device_orchestrator.h"

DeviceOrchestrator::DeviceOrchestrator()
: Node("device_orchestrator")
{
    initParameters();

    // Action Clients
    motorScanClient_ = rclcpp_action::create_client<MotorScan>(this, "motor_scan");

    // Service Clients
    startScanServiceTransformer_ = this->create_client<StartScanService>("start_scan_transformer");
    stopScanServiceTransformer_ = this->create_client<StopScanService>("stop_scan_transformer");

    // Create subscription to joy topic
    joySubscriber_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&DeviceOrchestrator::joyCallback, this, std::placeholders::_1)
    );
    
    robotController_ = new RobotController(this->get_logger());
    if (robotController_ == nullptr)
    {
        LOG_ROS_ERROR(this, "Failed to create robot controller");
        rclcpp::shutdown();
    }
    
    // TEMPORAL TEST CODE TO TEST MOTOR
    rclcpp::sleep_for(std::chrono::seconds(2));
    robotController_->move();
    rclcpp::sleep_for(std::chrono::seconds(1));
    robotController_->stop();
    // END TEST CODE
    
}

void DeviceOrchestrator::initParameters ()
{
    // Empty for now.
    // @todo -> move some lidar_rotation_motor parameters here and pass them via MotorScan
    return;
}

void DeviceOrchestrator::initScan ()
{
    LOG_ROS_ERROR(this, "Initializing scanning process");

    initScanTransformer();

    inScan_ = true;
}

void DeviceOrchestrator::stopScan (EndScanReason endReason)
{
    LOG_ROS_ERROR(this, "Stop scanning process due to reason %d", endReason);

    // Handle request to cancel the MotorScan goal
    if (motorScanGoalHandle_ && endReason == EndScanReason::CANCEL)
    {
        LOG_ROS_INFO(this, "Cancelling MotorScan action");
        auto cancelFuture = motorScanClient_->async_cancel_goal(
          motorScanGoalHandle_,
          std::bind(&DeviceOrchestrator::stopMotorScanCallback, this, std::placeholders::_1)
        );
    }
    stopScanTransformer(endReason);

    inScan_ = false;
}

Result DeviceOrchestrator::initScanTransformer ()
{
    Result res = RESULT_OK;

    // Wait for the service to be available
    if (!startScanServiceTransformer_->wait_for_service(std::chrono::seconds(5))) 
    {
        LOG_ROS_ERROR(this, "Transformer start scan service not available after waiting");
        return res;
    }

    auto request = std::make_shared<StartScanService::Request>();

    LOG_ROS_INFO(this, "Sending start scan request.");
    auto future = startScanServiceTransformer_->async_send_request(
        request, 
        std::bind(&DeviceOrchestrator::initScanTransformerCallback, this, std::placeholders::_1)
    );

    return res;
}

void DeviceOrchestrator::initScanTransformerCallback (rclcpp::Client<StartScanService>::SharedFuture startScanServiceFuture)
{
  auto response = startScanServiceFuture.get();
  if (!response->success)
  {
    LOG_ROS_ERROR(this, "Start scan service request failed");
    rclcpp::shutdown();
  }
}

Result DeviceOrchestrator::stopScanTransformer (EndScanReason endReason)
{
    Result res = RESULT_OK;

    // Wait for the service to be available
    if (!stopScanServiceTransformer_->wait_for_service(std::chrono::seconds(5))) 
    {
        LOG_ROS_ERROR(this, "Transformer stop scan service not available after waiting");
        return res;
    }

    auto request = std::make_shared<StopScanService::Request>();
    request->stop_reason = static_cast<int8_t>(endReason);

    LOG_ROS_INFO(this, "Sending stop scan request to transformer.");
    auto future = stopScanServiceTransformer_->async_send_request(
        request, 
        std::bind(&DeviceOrchestrator::stopScanTransformerCallback, this, std::placeholders::_1)
    );

    return res;
}
void DeviceOrchestrator::stopScanTransformerCallback (rclcpp::Client<StopScanService>::SharedFuture stopScanServiceFuture)
{
  auto response = stopScanServiceFuture.get();
  if (!response->success)
  {
    LOG_ROS_ERROR(this, "Stop scan service request failed");
    rclcpp::shutdown();
  }
}

void DeviceOrchestrator::startMotorScan()
{
    if (!motorScanClient_->wait_for_action_server()) {
        LOG_ROS_ERROR(this, "Action server not available after waiting");
        rclcpp::shutdown();
        return;
    }

    auto motorScanGoal = MotorScan::Goal();

    LOG_ROS_INFO(this, "Requesting motor to start the scan");

    auto motorScanRequestOptions = rclcpp_action::Client<MotorScan>::SendGoalOptions();
    motorScanRequestOptions.goal_response_callback = std::bind(&DeviceOrchestrator::motorScanGoalResponse, this, std::placeholders::_1);
    motorScanRequestOptions.feedback_callback = std::bind(&DeviceOrchestrator::motorScanFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    motorScanRequestOptions.result_callback = std::bind(&DeviceOrchestrator::motorScanResultCallback, this, std::placeholders::_1);

    auto goalHandleFuture = motorScanClient_->async_send_goal(motorScanGoal, motorScanRequestOptions);
    motorScanGoalRequested_ = true;
}

void DeviceOrchestrator::stopMotorScanCallback(const std::shared_ptr<action_msgs::srv::CancelGoal_Response> response)
{
    if (response->return_code == action_msgs::srv::CancelGoal_Response::ERROR_NONE)
    {
        LOG_ROS_INFO(this, "MotorScan action cancellation request accepted");
    }
    else
    {
        LOG_ROS_ERROR(this, "Unknown response code for MotorScan action cancellation request");
        rclcpp::shutdown();
    }
}

void DeviceOrchestrator::motorScanGoalResponse(const GoalHandleMotorScan::SharedPtr & goalHandle)
{
    if (!goalHandle) {
      LOG_ROS_ERROR(this, "Goal was rejected by server");
    } 
    else
    {
      LOG_ROS_INFO(this, "Goal accepted by server, waiting for result");
      motorScanGoalHandle_ = goalHandle;
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      initScan();
    }
    motorScanGoalRequested_ = false;
}

void DeviceOrchestrator::motorScanFeedbackCallback(GoalHandleMotorScan::SharedPtr goalHandle, const std::shared_ptr<const MotorScan::Feedback> feedback)
{
    //LOG_ROS_INFO(this, "Current scan angle: %f", feedback->angle);
    (void)goalHandle;
    (void)feedback;
}

void DeviceOrchestrator::motorScanResultCallback(const GoalHandleMotorScan::WrappedResult & result)
{
    LOG_ROS_INFO(this, "Got result from MotorScan action %d", static_cast<uint8_t>(result.code));
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        LOG_ROS_INFO(this, "Goal ended with result: %d", result.result->end_result);
        stopScan(static_cast<EndScanReason>(result.result->end_result));
        break;
      case rclcpp_action::ResultCode::CANCELED:
        LOG_ROS_INFO(this, "Goal canceled");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        LOG_ROS_INFO(this, "Goal aborted.");
        rclcpp::shutdown();
        break;
      default:
        LOG_ROS_INFO(this, "Unknown result code");
        return;
    }
}

void DeviceOrchestrator::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    JoyButton pressedButton = getPressedButton(msg);
    if (pressedButton !=  JoyButton::BUTTON_NONE)
    {
        LOG_ROS_INFO(this, "Pressed button: %d", static_cast<int>(pressedButton));
        switch (pressedButton)
        {
            case JoyButton::BUTTON_SCAN:
                LOG_ROS_INFO(this, "BUTTON_SCAN pressed");
                if (!inScan_ && !motorScanGoalRequested_)
                {
                    startMotorScan();
                }
                else
                {
                    LOG_ROS_ERROR(this, "Scan already in progress");
                }
                break;
            case JoyButton::BUTTON_CANCEL:
                LOG_ROS_INFO(this, "BUTTON_CANCEL pressed");
                if (inScan_ || motorScanGoalRequested_)
                {
                    stopScan(EndScanReason::CANCEL);
                }
                else
                {
                    LOG_ROS_ERROR(this, "No scan in progress to cancel");
                }
            break;
            default:
                LOG_ROS_INFO(this, "Unknown button pressed");
                break;
        }
    }

    JoyCoordinates axisInformation;
    axisInformation.x = (std::abs(msg->axes[0]) > AXIS_THRESHOLD) ? msg->axes[0] : 0;
    axisInformation.y = (std::abs(msg->axes[1]) > AXIS_THRESHOLD) ? msg->axes[1] : 0;
    LOG_ROS_INFO(this, "Axis x: %f, y: %f", axisInformation.x, axisInformation.y);


    // @todo - not ready
    //robotController_->move(axisInformation);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeviceOrchestrator>());
  rclcpp::shutdown();
  return 0;
}