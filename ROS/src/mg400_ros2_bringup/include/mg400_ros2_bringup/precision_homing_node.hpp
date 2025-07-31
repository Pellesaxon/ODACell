#ifndef PRECISION_HOMING_NODE_HPP_
#define PRECISION_HOMING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <atomic>
#include <vector>

#include "mg400_msgs/action/precision_homing.hpp"
#include "hg_c1030_msgs/action/control_streaming.hpp"
#include "mg400_msgs/action/move_to_joint.hpp" // The new action for synchronous moves
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "pid_controller.hpp"

class PrecisionHomingNode : public rclcpp::Node
{
public:
    explicit PrecisionHomingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~PrecisionHomingNode();

private:
    // Typedefs for convenience
    using HomingAction = mg400_msgs::action::PrecisionHoming;
    using GoalHandleHoming = rclcpp_action::ServerGoalHandle<HomingAction>;
    using LaserControlAction = hg_c1030_msgs::action::ControlStreaming;
    using MoveToJointAction = mg400_msgs::action::MoveToJoint;
    using GoalHandleMoveToJoint = rclcpp_action::ClientGoalHandle<MoveToJointAction>;

    // --- Callback groups ---
    rclcpp::CallbackGroup::SharedPtr action_server_cb_group_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;

    // --- Action Servers & Clients ---
    rclcpp_action::Server<HomingAction>::SharedPtr homing_action_server_;
    rclcpp_action::Client<LaserControlAction>::SharedPtr laser_control_client_;
    rclcpp_action::Client<MoveToJointAction>::SharedPtr move_to_joint_client_;

    // --- Subscriptions ---
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sensor_x_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sensor_y_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // --- PID Controllers & State ---
    PIDController pid_x_;
    PIDController pid_y_;
    std::atomic<double> current_distance_x_;
    std::atomic<double> current_distance_y_;

    // --- Homing Action Server Callbacks ---
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const HomingAction::Goal>);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleHoming>);
    void handle_accepted(const std::shared_ptr<GoalHandleHoming>);
    void execute_homing(const std::shared_ptr<GoalHandleHoming>);

    // --- Current Joint States ---
    std::mutex joint_state_mutex_;
    std::vector<double> current_joint_angles_;
    std::atomic<bool> has_received_joint_state_;

    // --- Sensor Callbacks ---
    void sensorXCallback(const sensor_msgs::msg::Range::SharedPtr msg);
    void sensorYCallback(const sensor_msgs::msg::Range::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // --- Helper Functions ---
    bool setLaserStreamingState(bool power_on);

    // --- Skrrr ---
    void execute_homing_pid(const std::shared_ptr<GoalHandleHoming> goal_handle);
    void execute_homing_skrrrrr(const std::shared_ptr<GoalHandleHoming> goal_handle);
};

#endif // PRECISION_HOMING_NODE_HPP_