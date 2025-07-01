/**
 * @file mg400_driver_node.cpp
 * @brief Implementation for the MG400DriverNode class.
 *
 * @version 1.1
 * @date 2025-06-25
 * @author LT
 */

#include "mg400_ros2_bringup/mg400_driver_node.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstring> // For memcpy
#include <regex>   // Error parsing

// Define constants from the documentation
constexpr int REALTIME_FEEDBACK_PORT = 30004;
constexpr int MOTION_COMMAND_PORT = 30003;
constexpr int DASHBOARD_COMMAND_PORT = 29999;
constexpr int FEEDBACK_PACKET_SIZE = 1440;

constexpr int COLLISION_LEVEL = 2; // Default collision level for fail-safe

// Helper to convert radians to degrees
double to_deg(double rad)
{
    return rad * 180.0 / M_PI;
}

namespace mg400_ros2_bringup
{

    MG400DriverNode::MG400DriverNode(const rclcpp::NodeOptions &options)
        : Node("mg400_driver_node", options), shm_bridge_("ERROR: No robot name provided")
    {

        robot_name_ = "mg400";
        robot_ip_ = "192.168.1.6";

        RCLCPP_INFO(this->get_logger(), "Starting MG400 driver for robot %s at IP: %s", robot_name_.c_str(), robot_ip_.c_str());

        shm_bridge_ = SharedMemoryBridge(robot_name_);

        initialize_error_maps();

        is_robot_connected_and_enabled_ = false;
        is_robot_in_error_ = false;


        // Create the shared memory
        if (!shm_bridge_.create())
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to create shared memory segment. Exiting.");
            throw std::runtime_error("Failed to create shared memory");
        }
        RCLCPP_INFO(this->get_logger(), "Shared memory created successfully.");
        // Initialize shared memory
        memset(shm_bridge_.shared_memory_ptr, 0, sizeof(RealTimeState));

        // Initialize ROS interfaces
        status_publisher_ = this->create_publisher<mg400_msgs::msg::RobotStatus>("mg400/robot_status", 10);
        dashboard_action_server_ = rclcpp_action::create_server<DashboardCommand>(
            this,
            "mg400/dashboard_command",
            std::bind(&MG400DriverNode::handle_dashboard_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MG400DriverNode::handle_dashboard_cancel, this, std::placeholders::_1),
            std::bind(&MG400DriverNode::handle_dashboard_accepted, this, std::placeholders::_1));

        clear_error_service_ = this->create_service<std_srvs::srv::Trigger>(
            "mg400/clear_error",
            std::bind(&MG400DriverNode::clear_error_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "ROS publishers and action server are ready.");

        // --- INITIAL CONNECTION CHECK ---
        RCLCPP_INFO(this->get_logger(), "Testing connection to robot at %s...", robot_ip_.c_str());
        std::string response = send_dashboard_command("GetErrorID()"); // A simple, harmless command to test the connection.

        if (response.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Could not establish initial connection to the robot. Please check the IP address and network connectivity.");
            throw std::runtime_error("Initial connection to robot failed");
        }
        RCLCPP_INFO(this->get_logger(), "Initial connection successful. Robot response: '%s'", response.c_str());

        // ON-CONNECT COMMANDS
        std::string clear_response = send_dashboard_command("ClearError()");

        // Step C: Verify if the clear command was successful.
        if (clear_response.rfind("0,", 0) == 0) 
        {
            RCLCPP_INFO(this->get_logger(), "Successfully cleared robot errors. The robot should now be ready.");
        }
        else 
        {
            RCLCPP_WARN(this->get_logger(), "Failed to clear robot errors. Response: '%s'. Manual intervention may be required.", clear_response.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Motion port connected. Enabling robot...");
        std::string enable_response = send_dashboard_command("EnableRobot()");
        if (enable_response.rfind("0,{},", 0) == 0)
        {
            is_robot_connected_and_enabled_ = true;
            RCLCPP_INFO(this->get_logger(), "Robot enabled successfully.");
        }
        else
        {
            is_robot_connected_and_enabled_ = false;
            RCLCPP_ERROR(this->get_logger(), "Failed to enable robot. Will not send motion commands.");
        }

        // Built in fail-safe collision checking sensitivity
        RCLCPP_INFO(this->get_logger(), "Setting collision level to %d.", COLLISION_LEVEL);
        std::string collision_level_response = send_dashboard_command("SetCollisionLevel(" + std::to_string(COLLISION_LEVEL) + ")");
        if (collision_level_response.rfind("0,{},", 0) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Collision level set to %d successfully.", COLLISION_LEVEL);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set collision level. Response: '%s'", collision_level_response.c_str());
        }

        error_query_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&MG400DriverNode::query_detailed_errors, this));

        // Start communication threads
        is_running_ = true;
        feedback_thread_ = std::thread(&MG400DriverNode::feedback_loop, this);
        motion_thread_ = std::thread(&MG400DriverNode::motion_command_loop, this);
        RCLCPP_INFO(this->get_logger(), "Communication threads started.");
    }

    MG400DriverNode::~MG400DriverNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down driver node. Disabling robot...");
        is_running_ = false;

        if (is_robot_connected_and_enabled_)
        {
            send_dashboard_command("DisableRobot()");
            RCLCPP_INFO(this->get_logger(), "Robot disabled.");
        }

        if (feedback_thread_.joinable())
        {
            feedback_thread_.join();
        }
        if (motion_thread_.joinable())
        {
            motion_thread_.join();
        }
        // Shared memory is cleaned up by its own destructor
    }

    std::string MG400DriverNode::send_dashboard_command(const std::string &command)
    {
        int sock = connect_socket(DASHBOARD_COMMAND_PORT);
        if (sock < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to dashboard port for command: %s", command.c_str());
            return "";
        }

        if (send(sock, command.c_str(), command.length(), 0) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send dashboard command: %s", command.c_str());
            close(sock);
            return "";
        }

        char buffer[1024] = {0};
        recv(sock, buffer, 1024, 0);
        close(sock);

        RCLCPP_INFO(this->get_logger(), "Sent '%s', received: '%s'", command.c_str(), buffer);
        return std::string(buffer);
    }

    void MG400DriverNode::clear_error_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Clear error service called.");
        std::string robot_response = send_dashboard_command("ClearError()");

        if (robot_response.empty())
        {
            response->success = false;
            response->message = "Failed to send command to robot.";
        }
        else
        {
            // Check if the response indicates success (ErrorID is 0)
            if (robot_response.rfind("0,{},", 0) == 0)
            {
                response->success = true;
                response->message = "Successfully cleared errors. Response: " + robot_response;
            }
            else
            {
                response->success = false;
                response->message = "Robot indicated an error in response: " + robot_response;
            }
        }
    }

    int MG400DriverNode::connect_socket(int port)
    {
        // ... implementation is the same as before ...
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0)
        {
            RCLCPP_ERROR(get_logger(), "Socket creation error for port %d", port);
            return -1;
        }

        sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);

        if (inet_pton(AF_INET, robot_ip_.c_str(), &serv_addr.sin_addr) <= 0)
        {
            RCLCPP_ERROR(get_logger(), "Invalid address/ Address not supported for port %d", port);
            close(sock);
            return -1;
        }

        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
            RCLCPP_ERROR(get_logger(), "Connection Failed to port %d", port);
            close(sock);
            return -1;
        }
        RCLCPP_INFO(get_logger(), "Successfully connected to port %d", port);
        return sock;
    }

    void MG400DriverNode::feedback_loop()
    {
        std::vector<char> buffer(FEEDBACK_PACKET_SIZE);

        while (is_running_)
        {
            if (feedback_sock_ < 0)
            {
                
                feedback_sock_ = connect_socket(REALTIME_FEEDBACK_PORT);
                if (feedback_sock_ < 0)
                {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
            }

            int bytes_read = recv(feedback_sock_, buffer.data(), FEEDBACK_PACKET_SIZE, 0);

            if (bytes_read == FEEDBACK_PACKET_SIZE)
            {
                parse_feedback_and_update(buffer);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Feedback socket disconnected or received incomplete packet. Reconnecting...");
                close(feedback_sock_);
                feedback_sock_ = -1;
                is_robot_connected_and_enabled_ = false;
            }
        }
        if (feedback_sock_ >= 0)
            close(feedback_sock_);
    }

    void MG400DriverNode::parse_feedback_and_update(const std::vector<char> &buffer)
    {

        if (!shm_bridge_.shared_memory_ptr)
            return;

        double q_actual_deg[6], qd_actual_deg[6];
        memcpy(q_actual_deg, &buffer[432], sizeof(q_actual_deg));
        memcpy(qd_actual_deg, &buffer[480], sizeof(qd_actual_deg));

        for (int i = 0; i < 5; ++i)
        {
            shm_bridge_.shared_memory_ptr->q_actual[i] = q_actual_deg[i] * M_PI / 180.0;
            shm_bridge_.shared_memory_ptr->qd_actual[i] = qd_actual_deg[i] * M_PI / 180.0;
        }

        auto status_msg = mg400_msgs::msg::RobotStatus();
        uint64_t robot_mode_val;
        memcpy(&robot_mode_val, &buffer[24], sizeof(robot_mode_val));
        status_msg.robot_mode = static_cast<int32_t>(robot_mode_val);

        char error_status;
        memcpy(&error_status, &buffer[1029], sizeof(error_status));
        status_msg.is_in_error = (error_status != 0);
        is_robot_in_error_ = (error_status != 0);

        char enable_status;
        memcpy(&enable_status, &buffer[1026], sizeof(enable_status));
        status_msg.is_enabled = (enable_status != 0);

        char drag_status;
        memcpy(&drag_status, &buffer[1027], sizeof(drag_status));
        status_msg.is_in_drag = (drag_status != 0);

        char brake_status_byte;
        memcpy(&brake_status_byte, &buffer[1025], sizeof(brake_status_byte));
        status_msg.brake_status.resize(4);
        status_msg.brake_status[0] = (brake_status_byte & (1 << 5));
        status_msg.brake_status[1] = (brake_status_byte & (1 << 4));
        status_msg.brake_status[2] = (brake_status_byte & (1 << 3));
        status_msg.brake_status[3] = (brake_status_byte & (1 << 2));

        status_publisher_->publish(status_msg);
    }

    void MG400DriverNode::motion_command_loop()
    {

        const std::vector<double> joint_vel_limits_rad_s = {
            300.0 * M_PI / 180.0, // j1
            300.0 * M_PI / 180.0, // j2
            300.0 * M_PI / 180.0, // j3
            300.0 * M_PI / 180.0, // j4
            300.0 * M_PI / 180.0  // j5
        };

        const std::vector<double> joint_accel_limits_rad_s2 = {
            300.0 * M_PI / 180.0, // j1 (max vel / 1.0s)
            300.0 * M_PI / 180.0, // j2
            300.0 * M_PI / 180.0, // j3
            300.0 * M_PI / 180.0, // j4
            300.0 * M_PI / 180.0  // j5
        };

        while (is_running_)
        {
            if (motion_sock_ < 0)
            {
                motion_sock_ = connect_socket(MOTION_COMMAND_PORT);
                if (motion_sock_ < 0)
                {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
            }

            if (shm_bridge_.shared_memory_ptr && shm_bridge_.shared_memory_ptr->new_command_flag)
            {

                if (!is_robot_connected_and_enabled_)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Robot is not enabled. Dropping motion command.");
                    shm_bridge_.shared_memory_ptr->new_command_flag = false; // Consume the flag anyway
                    continue;
                }
                if (is_robot_in_error_)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Robot is in error state. Dropping motion command. Please call /mg400/clear_error service.");
                    shm_bridge_.shared_memory_ptr->new_command_flag = false; // Consume the flag
                    continue;
                }

                // Calculate the SpeedJ parameter
                double max_speed_ratio = 0.0;
                for (size_t i = 0; i < 5; ++i)
                {
                    double cmd_vel = std::abs(shm_bridge_.shared_memory_ptr->qd_command[i]);
                    if (joint_vel_limits_rad_s[i] > 1e-6)
                    { // Avoid division by zero
                        max_speed_ratio = std::max(max_speed_ratio, cmd_vel / joint_vel_limits_rad_s[i]);
                    }
                }
                // Convert ratio (0.0-1.0) to percentage (1-100) and clamp it
                int speed_j_param = static_cast<int>(max_speed_ratio * 100.0);
                if (speed_j_param < 1)
                    speed_j_param = 1;
                if (speed_j_param > 100)
                    speed_j_param = 100;

                // Calculate the AccJ parameter
                double max_accel_ratio = 0.0;
                for (size_t i = 0; i < 5; ++i)
                {
                    double cmd_accel = std::abs(shm_bridge_.shared_memory_ptr->qdd_command[i]);
                    if (joint_accel_limits_rad_s2[i] > 1e-6)
                    {
                        max_accel_ratio = std::max(max_accel_ratio, cmd_accel / joint_accel_limits_rad_s2[i]);
                    }
                }
                int accel_j_param = static_cast<int>(max_accel_ratio * 100.0);
                if (accel_j_param < 1)
                    accel_j_param = 1;
                if (accel_j_param > 100)
                    accel_j_param = 100;

                char cmd_buffer[256];
                snprintf(cmd_buffer, sizeof(cmd_buffer), "JointMovJ(%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d)",
                         to_deg(shm_bridge_.shared_memory_ptr->q_command[0]),
                         to_deg(shm_bridge_.shared_memory_ptr->q_command[1]),
                         to_deg(shm_bridge_.shared_memory_ptr->q_command[2]),
                         to_deg(shm_bridge_.shared_memory_ptr->q_command[3]),
                         to_deg(shm_bridge_.shared_memory_ptr->q_command[4]),
                         speed_j_param,
                         accel_j_param);

                if (send(motion_sock_, cmd_buffer, strlen(cmd_buffer), 0) < 0)
                {
                    RCLCPP_WARN(get_logger(), "Failed to send motion command. Reconnecting...");
                    close(motion_sock_);
                    motion_sock_ = -1;
                }
                else
                {
                    shm_bridge_.shared_memory_ptr->new_command_flag = false;
                    char recv_buffer[1024];
                    recv(motion_sock_, recv_buffer, sizeof(recv_buffer) - 1, 0);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (motion_sock_ >= 0)
            close(motion_sock_);
    }

    rclcpp_action::GoalResponse MG400DriverNode::handle_dashboard_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const DashboardCommand::Goal>)
    {
        RCLCPP_INFO(get_logger(), "Received dashboard command goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse MG400DriverNode::handle_dashboard_cancel(const std::shared_ptr<GoalHandleDashboardCommand>)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel dashboard command goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void MG400DriverNode::handle_dashboard_accepted(const std::shared_ptr<GoalHandleDashboardCommand> goal_handle)
    {
        std::thread{std::bind(&MG400DriverNode::execute_dashboard_command, this, std::placeholders::_1), goal_handle}.detach();
    }

    void MG400DriverNode::execute_dashboard_command(const std::shared_ptr<GoalHandleDashboardCommand> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<DashboardCommand::Result>();
        auto feedback = std::make_shared<DashboardCommand::Feedback>();

        feedback->feedback_msg = "Sending command: " + goal->command;
        goal_handle->publish_feedback(feedback);

        std::string robot_response = send_dashboard_command(goal->command);

        if (robot_response.empty())
        {
            result->response = "Failed to send command to robot.";
            result->error_id = -1;
            goal_handle->abort(result);
            return;
        }

        result->response = robot_response;
        try
        {
            size_t comma_pos = robot_response.find(',');
            if (comma_pos != std::string::npos)
            {
                result->error_id = std::stoi(robot_response.substr(0, comma_pos));
            }
        }
        catch (...)
        {
            result->error_id = -99;
        }

        if (goal_handle->is_active())
        {
            goal_handle->succeed(result);
        }
    }

    void MG400DriverNode::initialize_error_maps()
    {
        return;
    }

    void MG400DriverNode::parse_detailed_error_string(const std::string &error_string, mg400_msgs::msg::RobotStatus &status_msg)
    {
        // Example string: "0,{[[11,12],[1004],[],[],[],[],[]]},GetErrorID();"
        std::regex re(R"(\{\[\[(.*)\],\[(.*)\],\[(.*)\],\[(.*)\],\[(.*)\],\[(.*)\],\[(.*)\]\]\})");
        std::smatch match;

        if (std::regex_search(error_string, match, re) && match.size() == 8)
        {
            // Group 1: Controller alarms
            std::string controller_alarms_str = match[1].str();
            std::stringstream ss(controller_alarms_str);
            std::string segment;
            while (std::getline(ss, segment, ','))
            {
                if (!segment.empty())
                {
                    try
                    {
                        int code = std::stoi(segment);
                        status_msg.controller_error_codes.push_back(code);
                        if (controller_alarm_map_.count(code))
                        {
                            status_msg.controller_error_strings.push_back(controller_alarm_map_.at(code));
                        }
                        else
                        {
                            status_msg.controller_error_strings.push_back("Unknown Controller Error");
                        }
                    }
                    catch (...)
                    { /* ignore stoi errors */
                    }
                }
            }

            // Groups 2-7: Servo alarms. Doc says last 4 are servos, but response shows 6. We'll parse all 6.
            for (size_t i = 2; i <= 7; ++i)
            {
                std::string servo_alarm_str = match[i].str();
                if (!servo_alarm_str.empty())
                {
                    try
                    {
                        int code = std::stoi(servo_alarm_str);
                        status_msg.servo_error_codes.push_back(code);
                        if (servo_alarm_map_.count(code))
                        {
                            status_msg.servo_error_strings.push_back("Servo " + std::to_string(i - 1) + ": " + servo_alarm_map_.at(code));
                        }
                        else
                        {
                            status_msg.servo_error_strings.push_back("Servo " + std::to_string(i - 1) + ": Unknown Servo Error");
                        }
                    }
                    catch (...)
                    { /* ignore stoi errors */
                    }
                }
            }
        }
    }

    void MG400DriverNode::query_detailed_errors()
    {
        // Only query if the robot is in an error state
        if (is_robot_in_error_.load())
        {
            RCLCPP_INFO(this->get_logger(), "Robot is in error state, querying for details...");
            std::string response = send_dashboard_command("GetErrorID()");
            if (!response.empty())
            {
                auto detailed_status_msg = mg400_msgs::msg::RobotStatus();
                parse_detailed_error_string(response, detailed_status_msg);

                // Log the errors found
                for (const auto &err_str : detailed_status_msg.controller_error_strings)
                {
                    RCLCPP_WARN(this->get_logger(), "Controller Error: %s", err_str.c_str());
                }
                for (const auto &err_str : detailed_status_msg.servo_error_strings)
                {
                    RCLCPP_WARN(this->get_logger(), "Servo Error: %s", err_str.c_str());
                }

                // We can publish these details on the status topic.
                // Note: This will be a partial message, only containing error info.
                // A more advanced implementation might merge this with the real-time status.
                status_publisher_->publish(detailed_status_msg);
            }
        }
    }

} // namespace mg400_ros2_bringup

// The main function remains here, as this file builds the executable
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        // If the constructor throws, this object is never fully created.
        auto driver_node = std::make_shared<mg400_ros2_bringup::MG400DriverNode>();
        rclcpp::spin(driver_node);
    }
    catch (const std::runtime_error &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Node initialization failed: %s. Shutting down.", e.what());
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutdown complete.");
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}