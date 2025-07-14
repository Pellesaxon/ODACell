#include "mg400_ros2_bringup/precision_homing_node.hpp" // Use your package name

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <algorithm> // For std::clamp
#include <cmath>     // For std::isinf

// Use a constexpr for compile-time constants instead of a C-style macro.
constexpr size_t K_NUM_JOINTS = 4;

PrecisionHomingNode::PrecisionHomingNode(const rclcpp::NodeOptions &options)
    : Node("precision_homing_node", options),
      pid_x_(declare_parameter<double>("pid.x.p", 0.05),
             declare_parameter<double>("pid.x.i", 0.01),
             declare_parameter<double>("pid.x.d", 0.005),
             this->get_clock()),
      pid_z_(declare_parameter<double>("pid.z.p", 0.05),
             declare_parameter<double>("pid.z.i", 0.01),
             declare_parameter<double>("pid.z.d", 0.005),
             this->get_clock())
{
    // Declare parameters that will be used by the node.
    this->declare_parameter<std::string>("robot_name", "mg400");
    this->declare_parameter<std::string>("move_group_name", "mg400_arm");
    target_distance_x_ = this->declare_parameter<double>("target_distance.x", 0.027);
    target_distance_z_ = this->declare_parameter<double>("target_distance.z", 0.027);
    position_tolerance_ = this->declare_parameter<double>("tolerance.position", 0.0005);
    max_correction_step_ = this->declare_parameter<double>("max_correction_step", 0.001);

    // Initialize ROS 2 clients, servers, and subscriptions.
    fjt_action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this, "/mg400_arm_controller/follow_joint_trajectory");
        
    sensor_x_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/distance/sensor_0", 10, std::bind(&PrecisionHomingNode::sensorXCallback, this, std::placeholders::_1));
    sensor_z_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/distance/sensor_1", 10, std::bind(&PrecisionHomingNode::sensorZCallback, this, std::placeholders::_1));
    
    homing_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/start_homing", std::bind(&PrecisionHomingNode::startHomingCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    control_streaming_client_ = rclcpp_action::create_client<hg_c1030_msgs::action::ControlStreaming>(
        this, "/control_streaming");

    // The control loop timer is created but not started until the homing service is called.
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&PrecisionHomingNode::controlLoop, this));
    control_loop_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "Precision Homing Node is ready. Call the '~/start_homing' service to begin.");
}

void PrecisionHomingNode::init()
{
    const auto move_group_name = this->get_parameter("move_group_name").as_string();
    const std::string robot_description_param = "robot_description";

    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface for group '%s' using parameter '%s'.",
                move_group_name.c_str(), robot_description_param.c_str());

    moveit::planning_interface::MoveGroupInterface::Options options(move_group_name, robot_description_param);
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), options);

    if (move_group_->getPlanningFrame().empty())
    {
        RCLCPP_FATAL(this->get_logger(), "MoveGroupInterface failed to initialize. Check that the 'robot_description' parameter is set and the group name '%s' exists.", move_group_name.c_str());
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface successfully initialized.");

    // Log joint information for debugging and verification.
    const auto& all_joint_names = move_group_->getJointNames();
    std::string all_joints_list_str;
    for (const auto& name : all_joint_names) { all_joints_list_str += " " + name; }
    RCLCPP_INFO(this->get_logger(), "Total joints in group '%s' kinematic chain (%zu): [%s ]",
                move_group_name.c_str(), all_joint_names.size(), all_joints_list_str.c_str());

    const auto& active_joint_names = move_group_->getActiveJoints();
    std::string active_joints_list_str;
    for (const auto& name : active_joint_names) { active_joints_list_str += " " + name; }
    RCLCPP_INFO(this->get_logger(), "ACTIVE joints for planning and control (%zu): [%s ]",
                active_joint_names.size(), active_joints_list_str.c_str());

    if (active_joint_names.size() != K_NUM_JOINTS)
    {
        RCLCPP_WARN(this->get_logger(), "Expected %zu active joints but found %zu. Check SRDF and URDF files.",
                    K_NUM_JOINTS, active_joint_names.size());
    }

    RCLCPP_INFO(this->get_logger(), "Precision Homing Node is fully initialized.");
}

void PrecisionHomingNode::setLaserStreamingState(bool power_on)
{
    if (!control_streaming_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(this->get_logger(), "Control streaming action server not available!");
        if (pending_homing_response_)
        {
            pending_homing_response_->success = false;
            pending_homing_response_->message = "Action server for laser control not available.";
            pending_homing_response_ = nullptr;
        }
        return;
    }

    auto goal_msg = hg_c1030_msgs::action::ControlStreaming::Goal();
    goal_msg.start_streaming = power_on;

    auto send_goal_options = rclcpp_action::Client<hg_c1030_msgs::action::ControlStreaming>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<hg_c1030_msgs::action::ControlStreaming>::SharedPtr &goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Laser control goal was rejected by server.");
            if (pending_homing_response_) {
                pending_homing_response_->success = false;
                pending_homing_response_->message = "Laser control goal was rejected.";
                pending_homing_response_ = nullptr;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Laser control goal accepted by server, waiting for result...");
        }
    };

    send_goal_options.result_callback =
        [this, power_on](const rclcpp_action::ClientGoalHandle<hg_c1030_msgs::action::ControlStreaming>::WrappedResult &result)
    {
        bool success = false;
        std::string message;

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Successfully set laser power %s.", power_on ? "ON" : "OFF");
            if (power_on && pending_homing_response_) {
                RCLCPP_INFO(this->get_logger(), "Laser is ON. Starting homing control loop.");
                pid_x_.reset();
                pid_z_.reset();
                is_homing_active_ = true;
                control_loop_timer_->reset();
                success = true;
                message = "Precision homing started.";
            }
            break;
        case rclcpp_action::ResultCode::ABORTED:
            message = "Laser control goal was aborted.";
            break;
        case rclcpp_action::ResultCode::CANCELED:
            message = "Laser control goal was canceled.";
            break;
        default:
            message = "Unknown error in laser control.";
            break;
        }

        if (!message.empty() && result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        }

        if (pending_homing_response_) {
            pending_homing_response_->success = success;
            pending_homing_response_->message = message;
            pending_homing_response_ = nullptr;
        }
    };

    control_streaming_client_->async_send_goal(goal_msg, send_goal_options);
}

void PrecisionHomingNode::startHomingCallback(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (is_homing_active_)
    {
        RCLCPP_WARN(this->get_logger(), "Homing is already active.");
        response->success = false;
        response->message = "Homing process is already running.";
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Request received to start homing. Turning on laser power...");
    pending_homing_response_ = response;
    setLaserStreamingState(true);
}

void PrecisionHomingNode::controlLoop()
{
    if (!is_homing_active_ || !move_group_) return;

    double error_x = target_distance_x_ - latest_distance_x_;
    double error_z = target_distance_z_ - latest_distance_z_;

    if (std::abs(error_x) < position_tolerance_ && std::abs(error_z) < position_tolerance_)
    {
        RCLCPP_INFO(this->get_logger(), "Homing successful! Position within tolerance.");
        is_homing_active_ = false;
        control_loop_timer_->cancel();
        setLaserStreamingState(false);
        return;
    }

    double correction_x = std::clamp(pid_x_.compute(error_x), -max_correction_step_, max_correction_step_);
    double correction_z = std::clamp(pid_z_.compute(error_z), -max_correction_step_, max_correction_step_);
    
    auto current_pose = move_group_->getCurrentPose().pose;
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x += correction_x;
    target_pose.position.z += correction_z;
    move_group_->setPoseTarget(target_pose);

    // To handle mimic/passive joints, we must filter the full joint solution from MoveIt
    // to get values for only the active, controllable joints.
    const auto& active_joint_names = move_group_->getActiveJoints();
    std::vector<double> all_target_joints;
    move_group_->getJointValueTarget(all_target_joints);

    const auto* joint_model_group = move_group_->getRobotModel()->getJointModelGroup(move_group_->getName());
    if (!joint_model_group) {
        RCLCPP_ERROR(this->get_logger(), "FATAL: Could not get JointModelGroup. Aborting.");
        is_homing_active_ = false;
        control_loop_timer_->cancel();
        return;
    }
    const auto& all_joint_names = joint_model_group->getJointModelNames();
    
    std::map<std::string, double> joint_value_map;
    for (size_t i = 0; i < all_joint_names.size(); ++i) {
        joint_value_map[all_joint_names[i]] = all_target_joints[i];
    }
    
    std::vector<double> active_target_joints;
    active_target_joints.reserve(active_joint_names.size());
    for (const auto& name : active_joint_names) {
        active_target_joints.push_back(joint_value_map.at(name));
    }

    sendCorrectionCommand(active_joint_names, active_target_joints);
}

void PrecisionHomingNode::sensorXCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    if (std::isinf(msg->range)) {
        if (is_homing_active_.load()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid 'inf' reading from X sensor. Aborting homing procedure.");
            is_homing_active_ = false;
            control_loop_timer_->cancel();
            setLaserStreamingState(false);
        }
        return; // Do not update state with an invalid value.
    }
    latest_distance_x_ = msg->range;
}

void PrecisionHomingNode::sensorZCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    if (std::isinf(msg->range)) {
        if (is_homing_active_.load()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid 'inf' reading from Z sensor. Aborting homing procedure.");
            is_homing_active_ = false;
            control_loop_timer_->cancel();
            setLaserStreamingState(false);
        }
        return; // Do not update state with an invalid value.
    }
    latest_distance_z_ = msg->range;
}

void PrecisionHomingNode::sendCorrectionCommand(const std::vector<std::string>& joint_names, const std::vector<double>& joint_positions)
{
    if (!fjt_action_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(this->get_logger(), "FollowJointTrajectory action server not available!");
        is_homing_active_ = false;
        control_loop_timer_->cancel();
        return;
    }

    control_msgs::action::FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory.joint_names = joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_positions;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    goal_msg.trajectory.points.push_back(point);

    fjt_action_client_->async_send_goal(goal_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto homing_node = std::make_shared<PrecisionHomingNode>(node_options);
    
    homing_node->init(); 

    executor.add_node(homing_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}