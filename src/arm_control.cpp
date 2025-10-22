/*
 * @Author: hejia
 * @Date: 2025-10-21 10:40:37
 * @LastEditTime: 2025-10-22 14:28:25
 * @Description: 
 * @FilePath: /src/arm_control/src/arm_control.cpp
 */
#include "arm_control.hpp"

/**
 * @description: 构造函数
 * @return {*}
 * @author: hejia
 */
arm_controller::arm_controller() : Node("arm_controller")
{
    this->declare_parameter("linear_step", 0.015);
    this->declare_parameter("angular_step", 0.05);
    this->declare_parameter("gripper_min", 0.0);
    this->declare_parameter("gripper_max", 0.08);
    this->declare_parameter("gripper_step", 0.01);
    this->declare_parameter("deadzone_linear", 0.1);
    this->declare_parameter("deadzone_angular", 0.1);

    linear_step_ = this->get_parameter("linear_step").as_double();
    angular_step_ = this->get_parameter("angular_step").as_double();
    min_gripper_ = this->get_parameter("gripper_min").as_double();
    max_gripper_ = this->get_parameter("gripper_max").as_double();
    gripper_step_ = this->get_parameter("gripper_step").as_double();
    deadzone_linear_ = this->get_parameter("deadzone_linear").as_double();
    deadzone_angular_ = this->get_parameter("deadzone_angular").as_double();

    if (min_gripper_ > max_gripper_) {
        std::swap(min_gripper_, max_gripper_);
    }

    {
        std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
        pos_cmd_state_ = piper_msgs::msg::PosCmd();
        pos_cmd_state_.gripper = std::clamp(0.0, min_gripper_, max_gripper_);
        pos_cmd_state_.mode1 = 0;
        pos_cmd_state_.mode2 = 0;
    }

    control_mode_ = CHASSIS;
    arm_state_ = ENABLED;

    enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/enable_flag", 10);
    pos_cmd_pub_ = this->create_publisher<piper_msgs::msg::PosCmd>("pos_cmd", 10);

    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::SensorDataQoS(), std::bind(&arm_controller::joy_callback, this, std::placeholders::_1));

    end_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>("end_pose", 10, std::bind(&arm_controller::end_pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Joystick arm controller started ");
}

/**
 * @description: 手柄回调函数
 * @param {SharedPtr} msg
 * @return {*}
 * @author: hejia
 */
void arm_controller::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    // 模式切换
    if (msg->buttons[BUTTON_SELECT])
    {
        switch_control_mode();
        return;
    }
        
    if (control_mode_ == ARM)
    {
        // 机械臂使能/失能
        if (msg->buttons[BUTTON_A])
        {
            if (arm_state_ == DISABLED)
                enable_arm();
            else 
                disable_arm();
            
            arm_state_ = (arm_state_ == DISABLED) ? ENABLED : DISABLED;
        }

        // 笛卡尔空间控制
        process_cartesian_input(*msg);
    }
}

/**
 * @description: 末端位姿回调函数
 * @param {SharedPtr} msg
 * @return {*}
 * @author: hejia
 */
void arm_controller::end_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
    pos_cmd_state_.x = msg->position.x;
    pos_cmd_state_.y = msg->position.y;
    pos_cmd_state_.z = msg->position.z;
    pos_cmd_state_.roll = roll;
    pos_cmd_state_.pitch = pitch;
    pos_cmd_state_.yaw = yaw;
}

/**
 * @description: 处理手柄笛卡尔空间输入
 * @param {Joy &} joy
 * @return {*}
 * @author: hejia
 */
void arm_controller::process_cartesian_input(const sensor_msgs::msg::Joy & joy)
{
    double delta_x = joy.axes[AXIS_LEFT_STICK_VERTICAL]>deadzone_linear_ ? joy.axes[AXIS_LEFT_STICK_VERTICAL] : 
                    (joy.axes[AXIS_LEFT_STICK_VERTICAL]<-deadzone_linear_ ? joy.axes[AXIS_LEFT_STICK_VERTICAL] : 0.0);
    double delta_y = joy.axes[AXIS_LEFT_STICK_HORIZONTAL]>deadzone_linear_ ? joy.axes[AXIS_LEFT_STICK_HORIZONTAL] : 
                    (joy.axes[AXIS_LEFT_STICK_HORIZONTAL]<-deadzone_linear_ ? joy.axes[AXIS_LEFT_STICK_HORIZONTAL] : 0.0);
    double delta_z = (-joy.axes[AXIS_L2] + 1) - (-joy.axes[AXIS_R2] + 1);
    int gripper_delta = (joy.buttons[BUTTON_L1] ? gripper_step_ : 0) -
                        (joy.buttons[BUTTON_R1] ? gripper_step_ : 0);

    if (delta_x == 0.0 && delta_y == 0.0 && delta_z == 0.0 && gripper_delta == 0)
        return;

    piper_msgs::msg::PosCmd cmd;
    {
        std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
        cmd = pos_cmd_state_;
    }

    cmd.x += delta_x * linear_step_;
    cmd.y += delta_y * linear_step_;
    cmd.z += delta_z * linear_step_;
    cmd.gripper = std::clamp(cmd.gripper + gripper_delta * gripper_step_, min_gripper_, max_gripper_);

    pos_cmd_pub_->publish(cmd);
}

/**
 * @description: 机械臂使能
 * @return {*}
 * @author: hejia
 */
void arm_controller::enable_arm()
{
    std_msgs::msg::Bool msg;
    msg.data = true;
    enable_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Arm enabled");
}

/**
 * @description: 机械臂失能
 * @return {*}
 * @author: hejia
 */
void arm_controller::disable_arm()
{
    std_msgs::msg::Bool msg;
    msg.data = false;
    enable_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Arm disabled");
}

/**
 * @description: 切换控制模式
 * @return {*}
 * @author: hejia
 */
void arm_controller::switch_control_mode()
{
    control_mode_ = (control_mode_ == ARM) ? CHASSIS : ARM;
    RCLCPP_INFO(get_logger(), "Control mode set to: %s", (control_mode_ == ARM) ? "ARM" : "CHASSIS");
}
