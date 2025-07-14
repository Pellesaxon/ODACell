#ifndef PRECISION_HOMING_NODE_HPP_
#define PRECISION_HOMING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <hg_c1030_msgs/action/control_streaming.hpp>

#include <atomic>
#include <memory>

// Very simple PID
struct PIDController {
    double kp, ki, kd;
    double integral = 0.0;
    double prev_error = 0.0;
    rclcpp::Clock::SharedPtr clock;
    rclcpp::Time last_time;
    bool first_run = true;

    PIDController(double p, double i, double d, rclcpp::Clock::SharedPtr c) : kp(p), ki(i), kd(d), clock(c) {}

    double compute(double error) {
        auto now = clock->now();
        if (first_run) {
            last_time = now;
            first_run = false;
        }
        
        double dt = (now - last_time).seconds();
        if (dt <= 0.0) return kp * error; // Avoid division by zero

        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        
        prev_error = error;
        last_time = now;

        return (kp * error) + (ki * integral) + (kd * derivative);
    }

    void reset() {
        integral = 0.0;
        prev_error = 0.0;
        first_run = true;
    }
};

class PrecisionHomingNode : public rclcpp::Node {
public:
    explicit PrecisionHomingNode(const rclcpp::NodeOptions& options);

    void init();

private:
    // --- Callbacks ---
    void sensorXCallback(const sensor_msgs::msg::Range::SharedPtr msg);
    void sensorZCallback(const sensor_msgs::msg::Range::SharedPtr msg);
    void startHomingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // --- Core Logic ---
    void controlLoop();
    void sendCorrectionCommand(const std::vector<std::string>& joint_names, const std::vector<double>& joint_positions);
        
    /**
     * @brief Asynchronously calls the laser control action server.
     * @param power_on True to start streaming, false to stop.
     */
    void setLaserStreamingState(bool power_on);

    // --- ROS Interfaces ---
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sensor_x_sub_, sensor_z_sub_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr fjt_action_client_;
    rclcpp_action::Client<hg_c1030_msgs::action::ControlStreaming>::SharedPtr control_streaming_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_service_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    
    // --- MoveIt ---
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_{nullptr};
    
    // --- State & Parameters ---
    std::atomic<double> latest_distance_x_{0.0};
    std::atomic<double> latest_distance_z_{0.0};
    std::atomic<bool> is_homing_active_{false};
    double target_distance_x_, target_distance_z_;
    double position_tolerance_;
    double max_correction_step_;

    // --- State for async operations ---
    // Store the service response while waiting for the laser action to complete.
    std::shared_ptr<std_srvs::srv::Trigger::Response> pending_homing_response_;

    // --- Controllers ---
    PIDController pid_x_;
    PIDController pid_z_;
};

#endif // PRECISION_HOMING_NODE_HPP_