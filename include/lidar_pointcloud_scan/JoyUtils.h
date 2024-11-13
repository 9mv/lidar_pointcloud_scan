#include "lidar_pointcloud_scan/types.h"
#include "sensor_msgs/msg/joy.hpp"

JoyButton getPressedButton (const sensor_msgs::msg::Joy::SharedPtr msg)
{
    JoyButton pressedButton = JoyButton::BUTTON_NONE;
    JoyButtonPriority highestPriority = JoyButtonPriority::PRIORITY_UNSET;

    // Handle multiple buttons pressed at the same time through a priority system
    for (size_t i = 0; i < msg->buttons.size(); ++i)
    {
        if (msg->buttons[i] == 1)
        {
            JoyButton currentButton = static_cast<JoyButton>(i);
            auto it = BUTTON_PRIORITY_MAP.find(currentButton);
            if (it != BUTTON_PRIORITY_MAP.end())
            {
                JoyButtonPriority currentPriority = it->second;
                if (static_cast<int>(currentPriority) > static_cast<int>(highestPriority))
                {
                    highestPriority = currentPriority;
                    pressedButton = currentButton;
                }
            }
        }
    }
    return pressedButton;
}