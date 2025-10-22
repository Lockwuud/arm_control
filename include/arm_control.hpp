/*
 * @Author: hejia
 * @Date: 2025-10-17 21:44:01
 * @LastEditTime: 2025-10-22 14:04:15
 * @Description: 
 * @FilePath: /src/arm_control/include/arm_control.hpp
 */
#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "piper_msgs/msg/pos_cmd.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class arm_controller : public rclcpp::Node
{
public:
    arm_controller();
    ~arm_controller() override = default;

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void end_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

    void process_cartesian_input(const sensor_msgs::msg::Joy & joy, const std::vector<int32_t> & previous_buttons);

    void enable_arm();
    void disable_arm();
    void switch_control_mode();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr end_pose_subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
    rclcpp::Publisher<piper_msgs::msg::PosCmd>::SharedPtr pos_cmd_pub_;

    piper_msgs::msg::PosCmd pos_cmd_state_;
    mutable std::mutex pos_cmd_mutex_;

    /* 参数预设 */
    double min_gripper_;             // 夹爪最小值
    double max_gripper_;             // 夹爪最大值
    double linear_step_;             // 线性步长
    double angular_step_;            // 角度步长
    double gripper_step_;            // 夹爪步长
    double deadzone_linear_;         // 摇杆线性死区
    double deadzone_angular_;        // 摇杆角度死区

    /*控制模式*/
    enum ControlMode {
        ARM,
        CHASSIS
    } control_mode_;

    /*机械臂状态*/
    enum ArmState {
        ENABLED,
        DISABLED
    } arm_state_;

    std::vector<int32_t> last_button_states_;

    /*手柄按键*/
    enum GamePadButtons {
        BUTTON_A = 0,           // 机械臂使能/失能
        BUTTON_B = 1,
        BUTTON_X = 3,
        BUTTON_Y = 4,
        BUTTON_L1= 6,           // 夹爪张开
        BUTTON_R1= 7,           // 夹爪闭合
        BUTTON_SELECT = 10,
        BUTTON_START = 11,
        BUTTON_L = 13,
        BUTTON_R = 14
    } gamepad_buttons_;

    /*手柄摇杆*/
    enum GamePadAxes {
        AXIS_LEFT_STICK_HORIZONTAL = 0,
        AXIS_LEFT_STICK_VERTICAL = 1,
        AXIS_RIGHT_STICK_HORIZONTAL = 2,
        AXIS_RIGHT_STICK_VERTICAL = 3,
        AXIS_L2 = 4,                    // 末端抬起
        AXIS_R2 = 5,                    // 末端下降
        AXIS_DPAD_HORIZONTAL = 6,
        AXIS_DPAD_VERTICAL = 7
    } gamepad_axes_;
};
