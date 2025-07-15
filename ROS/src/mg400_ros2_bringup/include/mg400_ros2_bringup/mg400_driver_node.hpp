#ifndef MG400_DRIVER_NODE_HPP
#define MG400_DRIVER_NODE_HPP

/**
 * @file mg400_driver_node.hpp
 * @brief Header for the MG400DriverNode class.
 *
 * This class encapsulates all logic for communicating with the Dobot MG400 robot
 * over TCP/IP. It provides a feedback loop for robot state and implements the
 * FollowJointTrajectory action server for MoveIt 2 integration.
 *
 * @version 2.0 (FollowJointTrajectory implementation)
 * @date 2025-07-03
 * @author LT
 */

#include <thread>
#include <string>
#include <vector>
#include <array>
#include <map>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mg400_msgs/action/dashboard_command.hpp"
#include "mg400_msgs/msg/robot_status.hpp"
#include "mg400_msgs/action/move_to_joint.hpp"
#include "mg400_ros2_bringup/alarms/mg400_alarm_manager.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp" 
#include "sensor_msgs/msg/joint_state.hpp"

/**
 * @brief Holds the structured response from a dashboard command.
 */
struct DashboardResponse
{
    bool success = false;
    int protocol_error_id = -999;
    std::string raw_response;
    std::string payload;
    const alarms::ErrorInfo *error_info = nullptr;
};

namespace mg400_ros2_bringup
{

    class MG400DriverNode : public rclcpp::Node
    {
    public:
        // Action definitions
        using DashboardCommand = mg400_msgs::action::DashboardCommand;
        using GoalHandleDashboardCommand = rclcpp_action::ServerGoalHandle<DashboardCommand>;
        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory; 
        using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
        using MoveToJointAction = mg400_msgs::action::MoveToJoint;
        using GoalHandleMoveToJoint = rclcpp_action::ServerGoalHandle<MoveToJointAction>;

        /**
         * @brief Construct a new MG400DriverNode object
         */
        MG400DriverNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        /**
         * @brief Destroy the MG400DriverNode object
         */
        ~MG400DriverNode();

        private:
        // --- Member Variables ---
        std::string robot_name_;
        std::string robot_ip_;
        std::atomic<bool> is_running_ = false;
        std::atomic<bool> is_robot_connected_and_enabled_ = false;
        std::atomic<bool> is_robot_in_error_ = false;

        // Robot state variables
        std::mutex robot_state_mutex_;
        std::vector<double> joint_positions_ = {0.0, 0.0, 0.0, 0.0}; // Joint positions in radians
        std::vector<double> joint_velocities_ = {0.0, 0.0, 0.0, 0.0}; // Joint velocities in radians/s
        std::atomic<bool> run_queued_cmd_flag_ = false;
        std::atomic<bool> queue_paused_flag_ = false; // Flag to pause the command queue
        uint64_t init_timestamp_ms_unix_ = 0; // Timestamp in milliseconds since epoch
        uint64_t init_wall_time_ms_unix_ = 0;

        // Sockets
        int feedback_sock_ = -1;
        int motion_sock_ = -1;
        std::mutex motion_socket_mutex_; // <<< NEW: To protect the motion socket

        // Communication threads
        std::thread feedback_thread_;

        // Periodically query detailed errors
        rclcpp::TimerBase::SharedPtr error_query_timer_;

        // ROS interfaces
        rclcpp::Publisher<mg400_msgs::msg::RobotStatus>::SharedPtr status_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

        // Action servers
        rclcpp_action::Server<DashboardCommand>::SharedPtr dashboard_action_server_;
        rclcpp_action::Server<FollowJointTrajectory>::SharedPtr fjt_action_server_;
        rclcpp_action::Server<MoveToJointAction>::SharedPtr move_to_joint_action_server_;

        // Services
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_error_service_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr auxiliary_power_service_;

        // --- Private Methods ---
        /**
         * @brief Establishes a TCP socket connection to a given port on the robot.
         * @param port The port number to connect to.
         * @return The socket file descriptor on success, -1 on failure.
         */
        int connect_socket(int port);

        /**
         * @brief Checks if the initial connection to the robot is successful.
         * @return True if the connection is established, false otherwise.
         */
        bool check_initial_connection();

        /**
         * @brief The main loop for the feedback thread. Continuously reads and parses data from port 30004.
         */
        void feedback_loop();

        void parse_detailed_error_string(const std::string &error_string, mg400_msgs::msg::RobotStatus &status_msg);

        /**
         * @brief Parses the 1440-byte feedback packet and updates shared memory and ROS publishers.
         * @param buffer The raw byte buffer received from the robot.
         */
        void parse_feedback_and_update(const std::vector<char> &buffer);

        /**
         * @brief Sends a high-level motion command to the robot's motion port (30003).
         * @param command The string command to send (e.g., "JointMovJ(...)").
         * @return True on success, false on failure.
         */
        bool send_motion_command(const std::string &command); // <<< NEW HELPER

        /**
         * @brief Sends a command to the dashboard port (29999).
         * @param command The string command to send, e.g., "EnableRobot()".
         * @return A DashboardResponse struct containing the result of the command execution.
         */
        DashboardResponse send_dashboard_command(const std::string &command);

        /**
         * @brief Service callback to handle requests to clear robot errors.
         */
        void clear_error_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        /**
         * @brief Service callback to handle requests to toggle auxiliary power (digital output).
         */
        void auxiliary_power_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

        /**
        * @brief Periodically called by a timer to query for detailed errors if the robot is in an error state.
        */
        void query_detailed_errors();
        
        // --- Dashboard Action Server Callbacks ---
        rclcpp_action::GoalResponse handle_dashboard_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const DashboardCommand::Goal>);
        rclcpp_action::CancelResponse handle_dashboard_cancel(const std::shared_ptr<GoalHandleDashboardCommand>);
        void handle_dashboard_accepted(const std::shared_ptr<GoalHandleDashboardCommand>);
        void execute_dashboard_command(const std::shared_ptr<GoalHandleDashboardCommand>);

        // --- FollowJointTrajectory Action Server Callbacks ---
        rclcpp_action::GoalResponse handle_fjt_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
        rclcpp_action::CancelResponse handle_fjt_cancel(const std::shared_ptr<GoalHandleFJT>);
        void handle_fjt_accepted(const std::shared_ptr<GoalHandleFJT>);
        void execute_trajectory(const std::shared_ptr<GoalHandleFJT> goal_handle);

        rclcpp_action::GoalResponse handle_move_to_joint_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToJointAction::Goal>);
        rclcpp_action::CancelResponse handle_move_to_joint_cancel(const std::shared_ptr<GoalHandleMoveToJoint>);
        void handle_move_to_joint_accepted(const std::shared_ptr<GoalHandleMoveToJoint>);
        void execute_move_to_joint(const std::shared_ptr<GoalHandleMoveToJoint>);
        
    };

} // namespace mg400_ros2_bringup

#endif // MG400_DRIVER_NODE_HPP