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

#define AUTO_HOME_ON_INIT false

#define DEFAULT_ROBOT_NAME "mg400"
#define DEFAULT_ROBOT_IP "192.168.1.6"

// Defined constants
#define REALTIME_FEEDBACK_PORT 30004
#define MOTION_COMMAND_PORT 30003
#define DASHBOARD_COMMAND_PORT 29999
#define FEEDBACK_PACKET_SIZE 1440

#define NUM_JOINTS 4      // Number of joints in the MG400 robot
#define COLLISION_LEVEL 2 // Default collision level for fail-safe
#define GLOBAL_SPEED_ACC_FACTOR 100
#define CONTINUOUS_PATH_SMOOTHING 40                // Default CP value for continuous path motion
#define TRAJECTORY_EXECUTION_AUTO_TIMEOUT_MULT 30.0 // Multiplier for trajectory execution timeout (very generous)
#define END_POS_DEG_TOLERANCE 0.01                  // Default end position tolerance in degrees
#define STARTED_MOVING_DEG_TOL 0.05

#define CONTINUOUS_PATH_RATIO 10
#define NO_PATH_SMOOTHING 0

constexpr std::array<const char *, 4> JOINT_NAMES = {"j1", "j2", "j3", "j4"};

enum MG400ControllerErrorCodes
{
    FAILED_TO_START_PATH_EXECUTION = -100,
    MOVED_TOO_SLOW_TIMEOUT = -101,
    FAILED_TO_SEND_CMD = -102,
};

// Helper to convert radians to degrees
static double to_deg(double rad)
{
    return rad * 180.0 / M_PI;
}

// Helper to convert degrees to radians
static double to_rad(double deg)
{
    return deg * M_PI / 180.0;
}

namespace mg400_ros2_bringup
{

    MG400DriverNode::MG400DriverNode(const rclcpp::NodeOptions &options)
        : Node("mg400_driver_node", options)
    {

        robot_name_ = this->declare_parameter<std::string>("robot_name", DEFAULT_ROBOT_NAME);
        robot_ip_ = this->declare_parameter<std::string>("robot_ip", DEFAULT_ROBOT_IP);

        RCLCPP_INFO(this->get_logger(), "Starting MG400 driver for robot %s at IP: %s", robot_name_.c_str(), robot_ip_.c_str());

        // Initialize ROS interfaces
        status_publisher_ = this->create_publisher<mg400_msgs::msg::RobotStatus>("/mg400/robot_status", 10);
        RCLCPP_INFO(this->get_logger(), "Robot status publisher initialized.");
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        RCLCPP_INFO(this->get_logger(), "Joint state publisher initialized.");

        dashboard_action_server_ = rclcpp_action::create_server<DashboardCommand>(
            this, "mg400/dashboard_command",
            std::bind(&MG400DriverNode::handle_dashboard_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MG400DriverNode::handle_dashboard_cancel, this, std::placeholders::_1),
            std::bind(&MG400DriverNode::handle_dashboard_accepted, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Dashboard command action server is ready.");

        fjt_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "mg400_arm_controller/follow_joint_trajectory",
            std::bind(&MG400DriverNode::handle_fjt_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MG400DriverNode::handle_fjt_cancel, this, std::placeholders::_1),
            std::bind(&MG400DriverNode::handle_fjt_accepted, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "FollowJointTrajectory action server is ready.");

        clear_error_service_ = this->create_service<std_srvs::srv::Trigger>(
            "mg400/clear_error",
            std::bind(&MG400DriverNode::clear_error_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Clear error service is ready.");

        auxiliary_power_service_ = this->create_service<std_srvs::srv::SetBool>(
            "mg400/auxiliary_power",
            std::bind(&MG400DriverNode::auxiliary_power_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Auxiliary power service is ready.");

        move_to_joint_action_server_ = rclcpp_action::create_server<MoveToJointAction>(
            this, "mg400/move_to_joint",
            std::bind(&MG400DriverNode::handle_move_to_joint_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MG400DriverNode::handle_move_to_joint_cancel, this, std::placeholders::_1),
            std::bind(&MG400DriverNode::handle_move_to_joint_accepted, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "MoveToJoint action server is ready.");

        RCLCPP_INFO(this->get_logger(), "ROS interfaces initialized.");

        // --- STARTUP SEQUENCE ---
        //

        bool initial_connection_success = check_initial_connection();
        if (!initial_connection_success)
        {
            RCLCPP_FATAL(this->get_logger(), "CRITICAL: Failed to establish initial connection to the robot. Driver cannot continue.");
            throw std::runtime_error("Failed to connect to robot on startup.");
        }

        RCLCPP_INFO(this->get_logger(), "Setting collision level to %d...", COLLISION_LEVEL);
        auto set_collision_response = send_dashboard_command("SetCollisionLevel(" + std::to_string(COLLISION_LEVEL) + ")");
        if (set_collision_response.success)
        {
            RCLCPP_INFO(this->get_logger(), "Collision level set to %d successfully.", COLLISION_LEVEL);
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "CRITICAL: Failed to set collision level. Reason: %s (ID: %d). Driver cannot continue.",
                         set_collision_response.error_info->en.description.data(), set_collision_response.protocol_error_id);
            throw std::runtime_error("Failed to set collision level on startup.");
        }

        RCLCPP_INFO(this->get_logger(), "Setting global speed factor...");
        auto set_speed_response = send_dashboard_command("SpeedFactor(" + std::to_string(GLOBAL_SPEED_ACC_FACTOR) + ")");
        if (set_speed_response.success)
        {
            RCLCPP_INFO(this->get_logger(), "Global speed factor set to %d successfully.", GLOBAL_SPEED_ACC_FACTOR);
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "CRITICAL: Failed to set global speed factor. Reason: %s (ID: %d). Driver cannot continue.",
                         set_speed_response.error_info->en.description.data(), set_speed_response.protocol_error_id);
            throw std::runtime_error("Failed to set global speed factor on startup.");
        }

        RCLCPP_INFO(this->get_logger(), "Setting global joint speed and acceleration parameters...");
        auto set_joint_speed_response = send_dashboard_command("SpeedJ(" + std::to_string(GLOBAL_SPEED_ACC_FACTOR) + ")");
        auto set_joint_acc_response = send_dashboard_command("AccJ(" + std::to_string(GLOBAL_SPEED_ACC_FACTOR) + ")");
        if (set_joint_speed_response.success && set_joint_acc_response.success)
        {
            RCLCPP_INFO(this->get_logger(), "Global joint speed and acceleration parameters set successfully.");
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "CRITICAL: Failed to set global joint speed or acceleration. Reason: %s (ID: %d). Driver cannot continue.",
                         set_joint_speed_response.error_info->en.description.data(),
                         set_joint_speed_response.protocol_error_id);
            throw std::runtime_error("Failed to set global joint speed or acceleration on startup.");
        }

        RCLCPP_INFO(this->get_logger(), "Clearing any previous errors...");
        auto clear_error_response = send_dashboard_command("ClearError()");
        if (clear_error_response.success)
        {
            RCLCPP_INFO(this->get_logger(), "Previous errors cleared successfully.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to clear previous errors. Reason: %s (ID: %d). Aborting.",
                        clear_error_response.error_info->en.description.data(),
                        clear_error_response.protocol_error_id);
            throw std::runtime_error("Failed to clear previous errors on startup.");
        }

        RCLCPP_INFO(this->get_logger(), "Enabling robot...");
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
        // --- END STARTUP ---

        error_query_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&MG400DriverNode::query_detailed_errors, this));

        // Start communication threads
        is_running_ = true;
        feedback_thread_ = std::thread(&MG400DriverNode::feedback_loop, this);
        // motion_thread_ is no longer needed

        motion_sock_ = connect_socket(MOTION_COMMAND_PORT);
        if (motion_sock_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to motion port on startup. Trajectory execution will fail.");
        }

#if AUTO_HOME_ON_INIT
        RCLCPP_INFO(this->get_logger(), "Auto-homing robot on startup...");
        send_dashboard_command("JointMovJ(0,0,0,0,100,100,10)");
#endif

        RCLCPP_INFO(this->get_logger(), "Startup sequence complete. Driver is running.");
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
        // No motion_thread_ to join

        // Close sockets
        if (motion_sock_ >= 0)
            close(motion_sock_);
        if (feedback_sock_ >= 0)
            close(feedback_sock_);
    }

    bool MG400DriverNode::check_initial_connection()
    {
        const int max_retries = 3;
        const int retry_delay_ms = 5000; // 2.5 seconds
        RCLCPP_INFO(this->get_logger(), "Checking initial connection to robot...");
        for (int attempt = 1; attempt <= max_retries; ++attempt)
        {
            auto response = send_dashboard_command("GetErrorID()");
            if (response.success)
            {
                RCLCPP_INFO(this->get_logger(), "Initial connection check successful. Robot is ready.");
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Initial connection check failed (attempt %d/%d). Reason: %s (ID: %d)",
                             attempt, max_retries,
                             response.error_info->en.description.data(), response.protocol_error_id);
                if (attempt < max_retries)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                }
            }
        }
        return false;
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

    bool MG400DriverNode::send_motion_command(const std::string &command)
    {
        std::lock_guard<std::mutex> lock(motion_socket_mutex_);

        // Check connection and reconnect if necessary
        if (motion_sock_ < 0)
        {
            RCLCPP_WARN(get_logger(), "Motion socket not connected. Attempting to reconnect...");
            motion_sock_ = connect_socket(MOTION_COMMAND_PORT);
            if (motion_sock_ < 0)
            {
                RCLCPP_ERROR(get_logger(), "Failed to reconnect motion socket. Cannot send command.");
                return false;
            }
        }

        // Send the command
        if (send(motion_sock_, command.c_str(), command.length(), 0) < 0)
        {
            RCLCPP_WARN(get_logger(), "Failed to send motion command. Closing socket for reconnection attempt on next call.");
            close(motion_sock_);
            motion_sock_ = -1;
            return false;
        }

        RCLCPP_INFO(get_logger(), "Sent new motion command: %s", command.c_str());
        return true;
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

    void MG400DriverNode::auxiliary_power_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                                   std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        const int aux_power_pin = 1;
        const std::string on_command = "DOExecute(" + std::to_string(aux_power_pin) + ",0)";
        const std::string off_command = "DOExecute(" + std::to_string(aux_power_pin) + ",1)";

        RCLCPP_INFO(this->get_logger(), "Auxiliary power service called with request: %s", request->data ? "ON" : "OFF");

        auto robot_response = send_dashboard_command(request->data ? on_command : off_command);

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
                response->message = request->data ? "Auxiliary power enabled." : "Auxiliary power disabled.";
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

        // Joint state parsing and publishing
        double q_actual_deg[NUM_JOINTS], qd_actual_deg[NUM_JOINTS];
        memcpy(q_actual_deg, &buffer[432], sizeof(q_actual_deg));
        memcpy(qd_actual_deg, &buffer[480], sizeof(qd_actual_deg));

        uint64_t timestamp_ms_unix;
        memcpy(&timestamp_ms_unix, &buffer[32], sizeof(timestamp_ms_unix));
        if (init_timestamp_ms_unix_ == 0 || init_wall_time_ms_unix_ == 0)
        {
            init_timestamp_ms_unix_ = timestamp_ms_unix;
            init_wall_time_ms_unix_ = this->get_clock().get()->now().nanoseconds() / 1e6; // Convert to milliseconds
            RCLCPP_INFO(get_logger(), "Initial wall time set to %lu ms since epoch.", init_wall_time_ms_unix_);
            RCLCPP_INFO(get_logger(), "Initial timestamp set to %lu ms since epoch.", init_timestamp_ms_unix_);
        }
        uint64_t elapsed_time_ms_unix_robot = timestamp_ms_unix - init_timestamp_ms_unix_;
        uint64_t current_timestamp_ms_unix = init_wall_time_ms_unix_ + elapsed_time_ms_unix_robot;

        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            joint_positions_[i] = to_rad(q_actual_deg[i]);
            joint_velocities_[i] = to_rad(qd_actual_deg[i]);
        }

        auto joint_state_msg = sensor_msgs::msg::JointState();
        // joint_state_msg.header.stamp = this->get_clock().get()->now(); // Try using robot's elapsed time, revert if synchronisation issues arise.
        joint_state_msg.header.stamp = rclcpp::Time(current_timestamp_ms_unix * 1e6, RCL_ROS_TIME);
        joint_state_msg.name = {JOINT_NAMES[0], JOINT_NAMES[1], JOINT_NAMES[2], JOINT_NAMES[3]};
        joint_state_msg.position = joint_positions_;
        joint_state_msg.velocity = joint_velocities_;
        joint_state_publisher_.get()->publish(joint_state_msg);

        // Robot status parsing and publishing
        char run_queued_cmd_val;
        memcpy(&run_queued_cmd_val, &buffer[1014], sizeof(run_queued_cmd_val));
        run_queued_cmd_flag_ = (run_queued_cmd_val != 0);

        char queue_paused_val;
        memcpy(&queue_paused_val, &buffer[1015], sizeof(queue_paused_val));
        queue_paused_flag_ = (queue_paused_val != 0);

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

    rclcpp_action::GoalResponse MG400DriverNode::handle_fjt_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received FollowJointTrajectory goal request.");

        if (goal->trajectory.points.empty())
        {
            RCLCPP_ERROR(get_logger(), "Rejecting FJT goal: Trajectory is empty.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->trajectory.joint_names.size() != NUM_JOINTS)
        {
            RCLCPP_ERROR(get_logger(), "Rejecting FJT goal: Incorrect number of joints. Expected %d, got %zu.", NUM_JOINTS, goal->trajectory.joint_names.size());
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Note: A more robust implementation would check if another goal is active.
        // For now, we accept any new goal, which will preempt the current one.
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void MG400DriverNode::handle_fjt_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        std::thread{std::bind(&MG400DriverNode::execute_trajectory, this, std::placeholders::_1), goal_handle}.detach();
    }

    rclcpp_action::CancelResponse MG400DriverNode::handle_fjt_cancel(
        const std::shared_ptr<GoalHandleFJT>)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel FJT goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void MG400DriverNode::execute_trajectory(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    // No changes to this section
    const std::vector<double> max_velocities = {5.23599, 5.23599, 5.23599, 5.23599};
    const std::vector<double> max_accelerations = {10.471975512, 10.471975512, 10.471975512, 10.471975512};

    RCLCPP_INFO(get_logger(), "Executing new trajectory with %zu points.", goal->trajectory.points.size());

    send_dashboard_command("ResetRobot()");
    std::vector<double> current_joint_positions = joint_positions_;
    RCLCPP_INFO(get_logger(), "Recorded current joint positions: [%.2f, %.2f, %.2f, %.2f]",
                to_deg(current_joint_positions[0]), to_deg(current_joint_positions[1]),
                to_deg(current_joint_positions[2]), to_deg(current_joint_positions[3]));

    // --- START REVISED LOGIC ---

    const auto& trajectory_points = goal->trajectory.points;
    const size_t num_points = trajectory_points.size();

    // Loop through all points and apply the correct CP value based on position
    for (size_t i = 0; i < num_points; ++i) {
        const auto& point = trajectory_points[i];
        
        // Cancellation and validity checks remain the same
        if (goal_handle->is_canceling()) { /* ... handle cancellation ... */ return; }
        if (point.velocities.empty() || point.accelerations.empty() || point.positions.size() != NUM_JOINTS) { /* ... handle error ... */ return; }

        // Calculate speed and acceleration percentages
        double speed_percent = 0.0, acceleration_percent = 0.0;
        for (size_t j = 0; j < NUM_JOINTS; j++) {
            speed_percent = std::max(speed_percent, (std::abs(point.velocities[j]) / max_velocities[j]) * 100.0);
            acceleration_percent = std::max(acceleration_percent, (std::abs(point.accelerations[j]) / max_accelerations[j]) * 100.0);
        }
        speed_percent = std::max(1.0, std::min(speed_percent, 100.0));
        acceleration_percent = std::max(1.0, std::min(acceleration_percent, 100.0));

        // Determine the correct CP value for this point
        int cp_value;
        if (num_points == 1) {
            // Case 1: Trajectory with only one point. Must be non-blended.
            cp_value = NO_PATH_SMOOTHING;
        } else if (i == 0) {
            // Case 2: The very first point of a multi-point trajectory. Must be non-blended.
            cp_value = NO_PATH_SMOOTHING;
        } else if (i == num_points - 1) {
            // Case 3: The very last point. Must be non-blended for accuracy.
            cp_value = NO_PATH_SMOOTHING;
        } else {
            // Case 4: An intermediate point. This is where we blend.
            cp_value = CONTINUOUS_PATH_RATIO;
        }

        // Format and send the command
        char cmd_buffer[256];
        snprintf(cmd_buffer, sizeof(cmd_buffer), "JointMovJ(%.4f,%.4f,%.4f,%.4f,SpeedJ=%d,AccJ=%d,CP=%d)",
                 to_deg(point.positions[0]), to_deg(point.positions[1]),
                 to_deg(point.positions[2]), to_deg(point.positions[3]),
                 static_cast<int>(speed_percent), static_cast<int>(acceleration_percent),
                 cp_value);

        if (!send_motion_command(std::string(cmd_buffer))) {
            RCLCPP_ERROR(get_logger(), "Failed to send motion command for point %zu. Aborting.", i);
            result->error_code = MG400ControllerErrorCodes::FAILED_TO_SEND_CMD;
            goal_handle->abort(result);
            return;
        }
    }

        RCLCPP_INFO(get_logger(), "All %zu points streamed. Waiting for motion to start...", goal->trajectory.points.size());

        // --- NEW TWO-STAGE WAITING LOGIC ---

        rclcpp::Time start_time = this->get_clock()->now();
        const auto &last_point = goal->trajectory.points.back();
        double expected_duration = rclcpp::Duration(last_point.time_from_start).seconds();
        double overall_timeout_s = expected_duration * TRAJECTORY_EXECUTION_AUTO_TIMEOUT_MULT;

        RCLCPP_INFO(get_logger(), "Expected trajectory duration: %.2f seconds. Overall timeout set to %.2f seconds.",
                    expected_duration, overall_timeout_s);

        double start_timeout_s = 5;
        bool robot_has_started_moving = false;
        while (rclcpp::ok() && !robot_has_started_moving)
        {

            // if robot has moved more than STARTED_MOVING_DEG_TOL degrees, we consider it started
            for (int i = 0; i < NUM_JOINTS; ++i)
            {
                if (std::abs(joint_positions_[i] - current_joint_positions[i]) > to_rad(STARTED_MOVING_DEG_TOL))
                {
                    robot_has_started_moving = true;
                    break;
                }
            }

            if (goal_handle->is_canceling())
            {
                // Handle cancellation...
                RCLCPP_INFO(get_logger(), "Trajectory canceled while waiting for motion to start.");
                send_dashboard_command("ResetRobot()");
                result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
                goal_handle->canceled(result);
                return;
            }
            if ((this->get_clock()->now() - start_time).seconds() > start_timeout_s)
            {
                RCLCPP_ERROR(get_logger(), "Timeout: Robot did not start moving. Aborting.");
                RCLCPP_ERROR(get_logger(), "Joint positions at timeout: [%.2f, %.2f, %.2f, %.2f]",
                             to_deg(joint_positions_[0]), to_deg(joint_positions_[1]),
                             to_deg(joint_positions_[2]), to_deg(joint_positions_[3]));
                send_dashboard_command("ResetRobot()");
                result->error_code = MG400ControllerErrorCodes::FAILED_TO_START_PATH_EXECUTION;
                result->error_string = "Robot did not start executing the motion within the timeout.";
                RCLCPP_ERROR(get_logger(), "Failed to start path execution. Aborting trajectory.");
                goal_handle->abort(result);
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        RCLCPP_INFO(get_logger(), "Motion has started. Now waiting for completion.");

        while (rclcpp::ok())
        {

            if (goal_handle->is_canceling())
            {
                RCLCPP_INFO(get_logger(), "Trajectory canceled while executing. Stopping robot.");
                send_dashboard_command("ResetRobot()");
                result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
                goal_handle->canceled(result);
                return;
            }

            if ((this->get_clock()->now() - start_time).seconds() > overall_timeout_s)
            {
                RCLCPP_ERROR(get_logger(), "Timeout waiting for trajectory execution. Stopping robot.");
                send_dashboard_command("ResetRobot()");
                result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
                goal_handle->abort(result);
                return;
            }

            bool all_joints_in_position = true; // Assume success
            {
                for (int i = 0; i < NUM_JOINTS; ++i)
                {
                    if (std::abs(joint_positions_[i] - last_point.positions[i]) > to_rad(END_POS_DEG_TOLERANCE))
                    {
                        all_joints_in_position = false; // If any joint is out, we have not succeeded
                        break;
                    }
                }
            }

            if (all_joints_in_position)
            {
                RCLCPP_INFO(get_logger(), "Final point reached. Trajectory successful.");
                result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
                goal_handle->succeed(result);
                return;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
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

    rclcpp_action::GoalResponse MG400DriverNode::handle_move_to_joint_goal(
        const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToJointAction::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received MoveToJoint goal request.");
        if (goal->joint_angles.size() != NUM_JOINTS)
        {
            RCLCPP_ERROR(get_logger(), "Rejecting goal: Incorrect number of joints. Expected %d, got %zu.", NUM_JOINTS, goal->joint_angles.size());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse MG400DriverNode::handle_move_to_joint_cancel(
        const std::shared_ptr<GoalHandleMoveToJoint>)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel MoveToJoint goal.");
        send_dashboard_command("ResetRobot()");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void MG400DriverNode::handle_move_to_joint_accepted(const std::shared_ptr<GoalHandleMoveToJoint> goal_handle)
    {
        std::thread{std::bind(&MG400DriverNode::execute_move_to_joint, this, std::placeholders::_1), goal_handle}.detach();
    }

    void MG400DriverNode::execute_move_to_joint(const std::shared_ptr<GoalHandleMoveToJoint> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<MoveToJointAction::Result>();
        auto feedback = std::make_shared<MoveToJointAction::Feedback>();

        RCLCPP_INFO(this->get_logger(), "Executing MoveToJoint goal: [%.2f, %.2f, %.2f, %.2f]",
                    to_deg(goal->joint_angles[0]), to_deg(goal->joint_angles[1]), to_deg(goal->joint_angles[2]), to_deg(goal->joint_angles[3]));

        // 1. Format the command
        char cmd_buffer[256];
        snprintf(cmd_buffer, sizeof(cmd_buffer), "JointMovJ(%.4f,%.4f,%.4f,%.4f,SpeedJ=%d,AccJ=%d,CP=0)",
                 to_deg(goal->joint_angles[0]), to_deg(goal->joint_angles[1]),
                 to_deg(goal->joint_angles[2]), to_deg(goal->joint_angles[3]),
                 static_cast<int>(goal->speed_percent), static_cast<int>(goal->acc_percent));

        // 2. Send the motion command
        if (!send_motion_command(std::string(cmd_buffer)))
        {
            result->error_code = -1;
            result->error_string = "Failed to send motion command to robot.";
            goal_handle->abort(result);
            return;
        }

        // 3. Monitor for completion
        rclcpp::Rate loop_rate(100);             // Check status at 100 Hz
        const double goal_tolerance_rad = 0.001; // ~0.05 degrees tolerance
        int stable_counter = 0;
        const int stability_threshold = 20; // Must be stable for 20*10ms = 200ms

        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling())
            {
                RCLCPP_INFO(get_logger(), "MoveToJoint goal canceled.");
                result->error_code = 1; // CANCELED
                result->error_string = "Goal was canceled by client.";
                goal_handle->canceled(result);
                return;
            }

            // Get current state from the feedback loop's shared variables
            std::vector<double> current_positions;
            current_positions = joint_positions_; // Copy current joint positions

            // Check if we have reached the destination
            bool all_joints_in_position = true;

            for (size_t i = 0; i < NUM_JOINTS; ++i)
            {
                feedback->current_angles[i] = current_positions[i];
                feedback->distance_to_goal[i] = std::abs(goal->joint_angles[i] - current_positions[i]);
                if (feedback->distance_to_goal[i] > goal_tolerance_rad)
                {
                    all_joints_in_position = false;
                }
            }
            goal_handle->publish_feedback(feedback);

            if (all_joints_in_position)
            {
                stable_counter++;
                if (stable_counter >= stability_threshold)
                {
                    RCLCPP_INFO(get_logger(), "MoveToJoint goal succeeded.");
                    result->error_code = 0; // SUCCESS
                    result->error_string = "Successfully reached target joint configuration.";
                    goal_handle->succeed(result);
                    return;
                }
            }
            else
            {
                stable_counter = 0; // Reset counter if we move out of tolerance
            }

            loop_rate.sleep();
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