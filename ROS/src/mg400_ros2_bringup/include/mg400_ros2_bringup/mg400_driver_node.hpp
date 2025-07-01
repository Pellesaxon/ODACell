#ifndef MG400_DRIVER_NODE_HPP
#define MG400_DRIVER_NODE_HPP

/**
 * @file mg400_driver_node.hpp
 * @brief Header for the MG400DriverNode class.
 *
 * This class encapsulates all logic for communicating with the Dobot MG400 robot
 * over TCP/IP and bridging its state to the ros2_control framework via shared memory.
 *
 * @version 1.1
 * @date 2025-06-25
 * @author LT
 */

#include <thread>
#include <string>
#include <vector>
#include <map>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mg400_ros2_bringup/shared_memory_bridge.hpp"
#include "mg400_msgs/action/dashboard_command.hpp"
#include "mg400_msgs/msg/robot_status.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mg400_ros2_bringup
{

    class MG400DriverNode : public rclcpp::Node
    {
    public:
        using DashboardCommand = mg400_msgs::action::DashboardCommand;
        using GoalHandleDashboardCommand = rclcpp_action::ServerGoalHandle<DashboardCommand>;

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
        SharedMemoryBridge shm_bridge_;
        std::atomic<bool> is_running_;

        // Sockets
        int feedback_sock_ = -1;
        int motion_sock_ = -1;

        // Communication threads
        std::thread feedback_thread_;
        std::thread motion_thread_;

        std::atomic<bool> is_robot_connected_and_enabled_;
        std::atomic<bool> is_robot_in_error_;

        std::map<int, std::string> controller_alarm_map_;
        std::map<int, std::string> servo_alarm_map_;

        // Periodically query detailed errors
        rclcpp::TimerBase::SharedPtr error_query_timer_;

        // ROS interfaces
        rclcpp::Publisher<mg400_msgs::msg::RobotStatus>::SharedPtr status_publisher_;
        rclcpp_action::Server<DashboardCommand>::SharedPtr dashboard_action_server_;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_error_service_;

        // --- Private Methods ---
        /**
         * @brief Establishes a TCP socket connection to a given port on the robot.
         * @param port The port number to connect to.
         * @return The socket file descriptor on success, -1 on failure.
         */
        int connect_socket(int port);

        /**
         * @brief The main loop for the feedback thread. Continuously reads and parses data from port 30004.
         */
        void feedback_loop();

        /**
         * @brief Parses the 1440-byte feedback packet and updates shared memory and ROS publishers.
         * @param buffer The raw byte buffer received from the robot.
         */
        void parse_feedback_and_update(const std::vector<char> &buffer);

        /**
         * @brief The main loop for the motion command thread. Watches for new commands in shared memory and sends them to port 30003.
         */
        void motion_command_loop();

        /**
         * @brief Sends a command to the dashboard port (29999).
         * @param command The string command to send, e.g., "EnableRobot()".
         * @return The robot's response string. Returns an empty string on failure.
         */
        std::string send_dashboard_command(const std::string &command);

        /**
         * @brief Service callback to handle requests to clear robot errors.
         */
        void clear_error_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        /**
         * @brief Populates the internal maps with known error codes and descriptions.
         */
        void initialize_error_maps();

        /**
         * @brief Parses the complex string response from a GetErrorID command.
         * @param error_string The raw string response from the robot.
         * @param status_msg The RobotStatus message to populate.
         */
        void parse_detailed_error_string(const std::string &error_string, mg400_msgs::msg::RobotStatus &status_msg);

        /**
         * @brief Periodically called by a timer to query for detailed errors if the robot is in an error state.
         */
        void query_detailed_errors();

        // --- Dashboard Action Server Callbacks ---
        rclcpp_action::GoalResponse handle_dashboard_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const DashboardCommand::Goal>);
        rclcpp_action::CancelResponse handle_dashboard_cancel(const std::shared_ptr<GoalHandleDashboardCommand>);
        void handle_dashboard_accepted(const std::shared_ptr<GoalHandleDashboardCommand>);
        void execute_dashboard_command(const std::shared_ptr<GoalHandleDashboardCommand>);
    };

} // namespace mg400_ros2_bringup

#endif // MG400_DRIVER_NODE_HPP