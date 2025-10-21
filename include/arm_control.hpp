#pragma once

#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "piper_msgs/msg/pos_cmd.hpp"

class arm_controller : public rclcpp::Node
{
public:
    arm_controller();
    ~arm_controller() override = default;

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void end_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

    void process_cartesian_input(const sensor_msgs::msg::Joy & joy);
    void process_joint_input(const sensor_msgs::msg::Joy & joy);

    void modify_and_publish_pos_cmd(const std::function<void(piper_msgs::msg::PosCmd &)> & modifier);
    void publish_pos_cmd();
    void toggle_control_mode();
    void enable_arm();
    void disable_arm();
    void control_gripper(double delta);

    double get_axis_value(const sensor_msgs::msg::Joy & joy, int axis_index) const;
    bool is_button_rising(const sensor_msgs::msg::Joy & joy, int button_index);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr end_pose_subscription_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
    rclcpp::Publisher<piper_msgs::msg::PosCmd>::SharedPtr pos_cmd_pub_;

    piper_msgs::msg::PosCmd pos_cmd_state_;
    mutable std::mutex pos_cmd_mutex_;

    std::vector<int> last_buttons_; 

    /* 参数预设 */
    std::string control_mode_;
    double joint_velocity_;          
    double min_gripper_;
    double max_gripper_;
    double linear_step_;
    double angular_step_;
    double gripper_step_;
    double deadzone_linear_;
    double deadzone_angular_;

    int axis_linear_x_;  
    int axis_linear_y_;
    int axis_linear_z_;
    int axis_angular_roll_;
    int axis_angular_pitch_;
    int axis_angular_yaw_;
    int axis_joint_1_;
    int axis_joint_2_;
    int axis_joint_3_;
    int axis_joint_4_;
    int axis_joint_5_;
    int axis_joint_6_;

    int button_enable_;
    int button_disable_;
    int button_toggle_mode_;
    int button_gripper_open_;
    int button_gripper_close_;
};
