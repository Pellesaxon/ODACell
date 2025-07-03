/**
 * @file mg400_driver_node.cpp
 * @brief Implementation for the MG400DriverNode class.
 *
 * @version 1.1 (Updated with constexpr alarm manager)
 * @date 2025-07-02
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
        is_robot_connected_and_enabled_ = false;
        is_robot_in_error_ = false;

        // Create the shared memory
        if (!shm_bridge_.create())
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to create shared memory segment. Exiting.");
            throw std::runtime_error("Failed to create shared memory");
        }
        RCLCPP_INFO(this->get_logger(), "Shared memory created successfully.");
        memset(shm_bridge_.shared_memory_ptr, 0, sizeof(RealTimeState));

        // Initialize ROS interfaces (this part is unchanged)
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

        // --- STARTUP SEQUENCE ---
        // Each step will now check the 'success' flag from the structured response.

        RCLCPP_INFO(this->get_logger(), "Step 1: Testing connection to robot at %s...", robot_ip_.c_str());
        auto test_conn_response = send_dashboard_command("GetErrorID()");
        if (!test_conn_response.success && test_conn_response.protocol_error_id == -999)
        {
            RCLCPP_FATAL(this->get_logger(), "CRITICAL: Could not establish initial connection. Reason: %s",
                         test_conn_response.error_info->en.description.data());
            throw std::runtime_error("Initial connection to robot failed");
        }
        RCLCPP_INFO(this->get_logger(), "Initial connection successful.");

        RCLCPP_INFO(this->get_logger(), "Step 2: Clearing any pre-existing errors...");
        auto clear_response = send_dashboard_command("ClearError()");
        if (!clear_response.success)
        {
            // This is a warning because the robot might just not have had any errors to clear.
            RCLCPP_WARN(this->get_logger(), "Could not clear errors. This may be normal if no errors were present.");
        }

        RCLCPP_INFO(this->get_logger(), "Step 3: Enabling robot...");
        auto enable_response = send_dashboard_command("EnableRobot()");
        if (enable_response.success)
        {
            is_robot_connected_and_enabled_ = true;
            RCLCPP_INFO(this->get_logger(), "Robot enabled successfully.");
        }
        else
        {
            is_robot_connected_and_enabled_ = false;
            RCLCPP_FATAL(this->get_logger(), "CRITICAL: Failed to enable robot. Reason: %s (ID: %d). Driver cannot continue.",
                         enable_response.error_info->en.description.data(), enable_response.protocol_error_id);
            throw std::runtime_error("Failed to enable robot on startup.");
        }

        RCLCPP_INFO(this->get_logger(), "Step 4: Setting collision level to %d.", COLLISION_LEVEL);
        auto collision_response = send_dashboard_command("SetCollisionLevel(" + std::to_string(COLLISION_LEVEL) + ")");
        if (!collision_response.success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set collision level. Reason: %s (ID: %d).",
                         collision_response.error_info->en.description.data(), collision_response.protocol_error_id);
        }

        // --- STARTUP COMPLETE ---

        error_query_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&MG400DriverNode::query_detailed_errors, this));

        is_running_ = true;
        feedback_thread_ = std::thread(&MG400DriverNode::feedback_loop, this);
        motion_thread_ = std::thread(&MG400DriverNode::motion_command_loop, this);
        RCLCPP_INFO(this->get_logger(), "Startup sequence complete. Communication threads started.");
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
    }

    DashboardResponse MG400DriverNode::send_dashboard_command(const std::string &command)
    {
        DashboardResponse response;

        int sock = connect_socket(DASHBOARD_COMMAND_PORT);
        if (sock < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to dashboard for command: '%s'", command.c_str());
            response.error_info = &alarms::getErrorInfo(alarms::Source::Protocol, response.protocol_error_id);
            response.raw_response = "Connection failed.";
            return response;
        }

        if (send(sock, command.c_str(), command.length(), 0) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send command '%s'", command.c_str());
            close(sock);
            response.error_info = &alarms::getErrorInfo(alarms::Source::Protocol, response.protocol_error_id);
            response.raw_response = "Send failed.";
            return response;
        }

        char buffer[1024] = {0};
        int bytes_received = recv(sock, buffer, 1024, 0);
        close(sock);

        if (bytes_received <= 0)
        {
            RCLCPP_WARN(this->get_logger(), "No response received for command '%s'", command.c_str());
            response.error_info = &alarms::getErrorInfo(alarms::Source::Protocol, response.protocol_error_id);
            response.raw_response = "No response received.";
            return response;
        }

        response.raw_response = std::string(buffer);

        // Now, parse the protocol error ID from the response
        try
        {
            size_t comma_pos = response.raw_response.find(',');
            if (comma_pos != std::string::npos)
            {
                response.protocol_error_id = std::stoi(response.raw_response.substr(0, comma_pos));
            }
            else
            {
                // If there's no comma, the response is malformed.
                response.protocol_error_id = -999;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not parse error ID from response: '%s'", response.raw_response.c_str());
            response.protocol_error_id = -999;
        }

        response.success = (response.protocol_error_id == 0);
        response.error_info = &alarms::getErrorInfo(alarms::Source::Protocol, response.protocol_error_id);

        // Log the outcome
        if (response.success)
        {
            RCLCPP_INFO(this->get_logger(), "Command '%s' successful. Response: %s", command.c_str(), response.raw_response.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Command '%s' failed. Reason: %s (ID: %d)",
                        command.c_str(),
                        response.error_info->en.description.data(),
                        response.protocol_error_id);
        }

        return response;
    }

    void MG400DriverNode::clear_error_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Clear error service called.");
        auto robot_response = send_dashboard_command("ClearError()");

        if (!robot_response.success)
        {
            response->success = false;
            response->message = "Failed to send command to robot.";
        }
        else
        {
            if (robot_response.success)
            {
                response->success = true;
                response->message = "Successfully cleared errors.";
            }
            else
            {
                std::string error_description = robot_response.error_info->en.description.data();
                response->success = false;
                response->message = "Robot indicated an error in response: " + error_description;
            }
        }
    }

    int MG400DriverNode::connect_socket(int port)
    {
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

        double q_actual_deg[NUM_JOINTS], qd_actual_deg[NUM_JOINTS];
        memcpy(q_actual_deg, &buffer[432], sizeof(q_actual_deg));
        memcpy(qd_actual_deg, &buffer[480], sizeof(qd_actual_deg));

        for (int i = 0; i < NUM_JOINTS; ++i)
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

        constexpr std::array<double, 4> joint_vel_limits_rad_s = {
            300.0 * M_PI / 180.0, 300.0 * M_PI / 180.0, 300.0 * M_PI / 180.0, 300.0 * M_PI / 180.0};

        constexpr std::array<double, 4> joint_accel_limits_rad_s2 = {
            300.0 * M_PI / 180.0, 300.0 * M_PI / 180.0, 300.0 * M_PI / 180.0, 300.0 * M_PI / 180.0};

        const int cp_ratio = 80;

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

                double max_speed_ratio = 0.0;
                for (size_t i = 0; i < NUM_JOINTS; ++i)
                {
                    double cmd_vel = std::abs(shm_bridge_.shared_memory_ptr->qd_command[i]);
                    if (joint_vel_limits_rad_s[i] > 1e-6)
                    {
                        max_speed_ratio = std::max(max_speed_ratio, cmd_vel / joint_vel_limits_rad_s[i]);
                    }
                }
                int speed_j_param = static_cast<int>(max_speed_ratio * 100.0);
                if (speed_j_param < 1)
                    speed_j_param = 0;
                if (speed_j_param > 100)
                    speed_j_param = 100;

                double max_accel_ratio = 0.0;
                for (size_t i = 0; i < NUM_JOINTS; ++i)
                {
                    double cmd_accel = std::abs(shm_bridge_.shared_memory_ptr->qdd_command[i]);
                    if (joint_accel_limits_rad_s2[i] > 1e-6)
                    {
                        max_accel_ratio = std::max(max_accel_ratio, cmd_accel / joint_accel_limits_rad_s2[i]);
                    }
                }
                int accel_j_param = static_cast<int>(max_accel_ratio * 100.0);
                if (accel_j_param < 1)
                    accel_j_param = 0;
                if (accel_j_param > 100)
                    accel_j_param = 100;

                char cmd_buffer[256];
                snprintf(cmd_buffer, sizeof(cmd_buffer), "JointMovJ(%.4f,%.4f,%.4f,%.4f,SpeedJ=%d,AccJ=%d,CP=%d)",
                         to_deg(shm_bridge_.shared_memory_ptr->q_command[0]),
                         to_deg(shm_bridge_.shared_memory_ptr->q_command[1]),
                         to_deg(shm_bridge_.shared_memory_ptr->q_command[2]),
                         to_deg(shm_bridge_.shared_memory_ptr->q_command[3]),
                         speed_j_param,
                         accel_j_param,
                         cp_ratio);

                // RCLCPP_INFO(get_logger(), "Sending motion command: %s", cmd_buffer);

                if (send(motion_sock_, cmd_buffer, strlen(cmd_buffer), 0) < 0)
                {
                    RCLCPP_WARN(get_logger(), "Failed to send motion command. Reconnecting...");
                    close(motion_sock_);
                    motion_sock_ = -1;
                }
                else
                {
                    shm_bridge_.shared_memory_ptr->new_command_flag = false;
                }

                RCLCPP_INFO(get_logger(), "Sent motion command: %s", cmd_buffer);
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

        // Call the new, smarter function that returns a structured response
        DashboardResponse robot_response = send_dashboard_command(goal->command);

        // The parsing is already done inside send_dashboard_command.
        // We just need to copy the results into our action result message.
        result->response = robot_response.raw_response;
        result->error_id = robot_response.protocol_error_id;

        if (goal_handle->is_active())
        {
            // Simply check the success flag from the response struct
            if (robot_response.success)
            {
                goal_handle->succeed(result);
            }
            else
            {
                // The warning/error message was already logged inside send_dashboard_command.
                // We just need to abort the action.
                goal_handle->abort(result);
            }
        }
    }

    void MG400DriverNode::parse_detailed_error_string(const std::string &error_string, mg400_msgs::msg::RobotStatus &status_msg)
    {

        RCLCPP_INFO(this->get_logger(), "Parsing detailed error string: %s", error_string.c_str());

        size_t start_pos = error_string.find('{');
        size_t end_pos = error_string.rfind('}');
        if (start_pos == std::string::npos || end_pos == std::string::npos || start_pos >= end_pos)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not find valid error block in response: %s", error_string.c_str());
            return;
        }
        std::string data_block = error_string.substr(start_pos, end_pos - start_pos + 1);

        data_block.erase(std::remove_if(data_block.begin(), data_block.end(),
                                        [](unsigned char c)
                                        { return std::isspace(c); }),
                         data_block.end());

        if (data_block == "{}")
        {
            RCLCPP_INFO(this->get_logger(), "Robot reports an error state but returned an empty detailed error list.");
            return; // This is a valid state, just nothing to do.
        }

        std::regex re(R"(\{\[\[(.*)\],\[(.*)\],\[(.*)\],\[(.*)\],\[(.*)\],\[(.*)\]\]\})");
        std::smatch match;

        if (std::regex_search(data_block, match, re) && match.size() == 7)
        {
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
                        const auto &error_info = alarms::getErrorInfo(alarms::Source::Controller, code);
                        status_msg.controller_error_strings.push_back(std::string(error_info.en.description));
                    }
                    catch (const std::exception &e)
                    { /* ignore stoi errors */
                    }
                }
            }

            for (size_t i = 2; i <= 6; ++i)
            {
                std::string servo_alarm_str = match[i].str();
                if (!servo_alarm_str.empty())
                {
                    try
                    {
                        int code = std::stoi(servo_alarm_str);
                        status_msg.servo_error_codes.push_back(code);
                        const auto &error_info = alarms::getErrorInfo(alarms::Source::Servo, code);
                        std::string msg = "Servo " + std::to_string(i - 1) + ": " + std::string(error_info.en.description);
                        status_msg.servo_error_strings.push_back(msg);
                    }
                    catch (const std::exception &e)
                    { /* ignore stoi errors */
                    }
                }
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Regex failed to parse sanitized error data block: %s", data_block.c_str());
        }
    }

    void MG400DriverNode::query_detailed_errors()
    {

        if (!is_robot_in_error_.load())
        {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Robot is in error state, querying for details...");

        DashboardResponse response = send_dashboard_command("GetErrorID()");

        if (response.success)
        {
            auto detailed_status_msg = mg400_msgs::msg::RobotStatus();
            parse_detailed_error_string(response.raw_response, detailed_status_msg);

            bool has_controller_errors = !detailed_status_msg.controller_error_codes.empty();
            bool has_servo_errors = !detailed_status_msg.servo_error_codes.empty();

            if (has_controller_errors)
            {
                for (size_t i = 0; i < detailed_status_msg.controller_error_codes.size(); ++i)
                {
                    RCLCPP_WARN(this->get_logger(), "Controller Error | ID: %d | Message: %s",
                                detailed_status_msg.controller_error_codes[i],
                                detailed_status_msg.controller_error_strings[i].c_str());
                }
            }

            if (has_servo_errors)
            {
                for (size_t i = 0; i < detailed_status_msg.servo_error_codes.size(); ++i)
                {
                    RCLCPP_WARN(this->get_logger(), "Servo Error | ID: %d | Message: %s",
                                detailed_status_msg.servo_error_codes[i],
                                detailed_status_msg.servo_error_strings[i].c_str());
                }
            }

            if (has_controller_errors || has_servo_errors)
            {
                status_publisher_->publish(detailed_status_msg);
            }
        }
    }


    } // namespace mg400_ros2_bringup

    int main(int argc, char *argv[])
    {
        rclcpp::init(argc, argv);
        try
        {
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