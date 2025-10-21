#include <algorithm>
#include <cmath>
#include <functional>

#include "arm_control.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

arm_controller::arm_controller() : Node("arm_controller")
{
    this->declare_parameter("control_mode", "cartesian");
    this->declare_parameter("linear_step", 0.01);
    this->declare_parameter("angular_step", 0.05);
    this->declare_parameter("joint_velocity", 0.3);
    this->declare_parameter("gripper_min", 0.0);
    this->declare_parameter("gripper_max", 0.08);
    this->declare_parameter("gripper_step", 0.01);
    this->declare_parameter("deadzone_linear", 0.1);
    this->declare_parameter("deadzone_angular", 0.1);

    this->declare_parameter("axis_linear_x", 1);
    this->declare_parameter("axis_linear_y", 0);
    this->declare_parameter("axis_linear_z", 7);
    this->declare_parameter("axis_angular_roll", 3);
    this->declare_parameter("axis_angular_pitch", 4);
    this->declare_parameter("axis_angular_yaw", 6);
    this->declare_parameter("axis_joint_1", 1);
    this->declare_parameter("axis_joint_2", 0);
    this->declare_parameter("axis_joint_3", 4);
    this->declare_parameter("axis_joint_4", 3);
    this->declare_parameter("axis_joint_5", 7);
    this->declare_parameter("axis_joint_6", 6);

    this->declare_parameter("button_enable", 4);
    this->declare_parameter("button_disable", 5);
    this->declare_parameter("button_toggle_mode", 3);
    this->declare_parameter("button_gripper_open", 0);
    this->declare_parameter("button_gripper_close", 1);

    control_mode_ = this->get_parameter("control_mode").as_string();
    linear_step_ = this->get_parameter("linear_step").as_double();
    angular_step_ = this->get_parameter("angular_step").as_double();
    joint_velocity_ = this->get_parameter("joint_velocity").as_double();
    min_gripper_ = this->get_parameter("gripper_min").as_double();
    max_gripper_ = this->get_parameter("gripper_max").as_double();
    gripper_step_ = this->get_parameter("gripper_step").as_double();
    deadzone_linear_ = this->get_parameter("deadzone_linear").as_double();
    deadzone_angular_ = this->get_parameter("deadzone_angular").as_double();

    axis_linear_x_ = this->get_parameter("axis_linear_x").as_int();
    axis_linear_y_ = this->get_parameter("axis_linear_y").as_int();
    axis_linear_z_ = this->get_parameter("axis_linear_z").as_int();
    axis_angular_roll_ = this->get_parameter("axis_angular_roll").as_int();
    axis_angular_pitch_ = this->get_parameter("axis_angular_pitch").as_int();
    axis_angular_yaw_ = this->get_parameter("axis_angular_yaw").as_int();
    axis_joint_1_ = this->get_parameter("axis_joint_1").as_int();
    axis_joint_2_ = this->get_parameter("axis_joint_2").as_int();
    axis_joint_3_ = this->get_parameter("axis_joint_3").as_int();
    axis_joint_4_ = this->get_parameter("axis_joint_4").as_int();
    axis_joint_5_ = this->get_parameter("axis_joint_5").as_int();
    axis_joint_6_ = this->get_parameter("axis_joint_6").as_int();

    button_enable_ = this->get_parameter("button_enable").as_int();
    button_disable_ = this->get_parameter("button_disable").as_int();
    button_toggle_mode_ = this->get_parameter("button_toggle_mode").as_int();
    button_gripper_open_ = this->get_parameter("button_gripper_open").as_int();
    button_gripper_close_ = this->get_parameter("button_gripper_close").as_int();

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

    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
        "/servo_server/delta_joint_cmds", 10);
    enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/enable_flag", 10);
    pos_cmd_pub_ = this->create_publisher<piper_msgs::msg::PosCmd>("pos_cmd", 10);

    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", rclcpp::SensorDataQoS(),
        std::bind(&arm_controller::joy_callback, this, std::placeholders::_1));

    end_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "end_pose", 10,
        std::bind(&arm_controller::end_pose_callback, this, std::placeholders::_1));

    last_buttons_.clear();

    RCLCPP_INFO(get_logger(), "Joystick arm controller started (mode: %s)", control_mode_.c_str());
}

void arm_controller::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    
}


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

void arm_controller::process_cartesian_input(const sensor_msgs::msg::Joy & joy)
{
    const auto get_linear = [&](int axis_index) {
        double value = get_axis_value(joy, axis_index);
        return (std::abs(value) >= deadzone_linear_) ? value : 0.0;
    };
    const auto get_angular = [&](int axis_index) {
        double value = get_axis_value(joy, axis_index);
        return (std::abs(value) >= deadzone_angular_) ? value : 0.0;
    };

    const double delta_x = get_linear(axis_linear_x_);
    const double delta_y = get_linear(axis_linear_y_);
    const double delta_z = get_linear(axis_linear_z_);
    const double delta_roll = get_angular(axis_angular_roll_);
    const double delta_pitch = get_angular(axis_angular_pitch_);
    const double delta_yaw = get_angular(axis_angular_yaw_);

    if (delta_x == 0.0 && delta_y == 0.0 && delta_z == 0.0 &&
        delta_roll == 0.0 && delta_pitch == 0.0 && delta_yaw == 0.0) {
        return;
    }

    modify_and_publish_pos_cmd([&](piper_msgs::msg::PosCmd & cmd) {
        cmd.x += delta_x * linear_step_;
        cmd.y += delta_y * linear_step_;
        cmd.z += delta_z * linear_step_;
        cmd.roll += delta_roll * angular_step_;
        cmd.pitch += delta_pitch * angular_step_;
        cmd.yaw += delta_yaw * angular_step_;
    });
}

void arm_controller::process_joint_input(const sensor_msgs::msg::Joy & joy)
{
    auto msg = control_msgs::msg::JointJog();
    msg.header.stamp = this->now();

    const auto push_joint = [&](int axis_index, const std::string & joint_name) {
        double value = get_axis_value(joy, axis_index);
        if (std::abs(value) >= deadzone_linear_) {
            msg.joint_names.push_back(joint_name);
            msg.velocities.push_back(value * joint_velocity_);
        }
    };

    push_joint(axis_joint_1_, "joint1");
    push_joint(axis_joint_2_, "joint2");
    push_joint(axis_joint_3_, "joint3");
    push_joint(axis_joint_4_, "joint4");
    push_joint(axis_joint_5_, "joint5");
    push_joint(axis_joint_6_, "joint6");

    if (!msg.joint_names.empty()) {
        joint_pub_->publish(msg);
    }
}

void arm_controller::modify_and_publish_pos_cmd(
    const std::function<void(piper_msgs::msg::PosCmd &)> & modifier)
{
    piper_msgs::msg::PosCmd cmd_copy;
    {
        std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
        modifier(pos_cmd_state_);
        cmd_copy = pos_cmd_state_;
    }
    pos_cmd_pub_->publish(cmd_copy);
}

void arm_controller::publish_pos_cmd()
{
    piper_msgs::msg::PosCmd cmd_copy;
    {
        std::lock_guard<std::mutex> lock(pos_cmd_mutex_);
        cmd_copy = pos_cmd_state_;
    }
    pos_cmd_pub_->publish(cmd_copy);
}

void arm_controller::toggle_control_mode()
{
    if (control_mode_ == "cartesian") {
        control_mode_ = "joint";
        RCLCPP_INFO(get_logger(), "Switched to joint-space control");
    } else {
        control_mode_ = "cartesian";
        RCLCPP_INFO(get_logger(), "Switched to cartesian control");
        publish_pos_cmd();
    }
}

void arm_controller::enable_arm()
{
    std_msgs::msg::Bool msg;
    msg.data = true;
    enable_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Arm enabled");
}

void arm_controller::disable_arm()
{
    std_msgs::msg::Bool msg;
    msg.data = false;
    enable_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Arm disabled");
}

void arm_controller::control_gripper(double delta)
{
    if (delta == 0.0) {
       return;
    }
    modify_and_publish_pos_cmd([&](piper_msgs::msg::PosCmd & cmd) {
        cmd.gripper = std::clamp(cmd.gripper + delta, min_gripper_, max_gripper_);
    });
}

double arm_controller::get_axis_value(const sensor_msgs::msg::Joy & joy, int axis_index) const
{
    if (axis_index < 0 || axis_index >= static_cast<int>(joy.axes.size())) {
        return 0.0;
    }
    return joy.axes[axis_index];
}

bool arm_controller::is_button_rising(const sensor_msgs::msg::Joy & joy, int button_index)
{
    if (button_index < 0 || button_index >= static_cast<int>(joy.buttons.size())) {
        return false;
    }
    int current = joy.buttons[button_index];
    int previous = 0;
    if (button_index < static_cast<int>(last_buttons_.size())) {
        previous = last_buttons_[button_index];
    }
    return current == 1 && previous == 0;
}
