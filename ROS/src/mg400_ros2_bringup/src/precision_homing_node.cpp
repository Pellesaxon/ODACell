// src/precision_homing_node.cpp

#include "mg400_ros2_bringup/precision_homing_node.hpp" // Use your package name
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <algorithm> // For std::clamp

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

    const auto robot_name = this->declare_parameter<std::string>("robot_name", "mg400");
    const auto move_group_name = this->declare_parameter<std::string>("move_group_name", "mg400_arm");

    target_distance_x_ = this->declare_parameter<double>("target_distance.x", 0.027);
    target_distance_z_ = this->declare_parameter<double>("target_distance.z", 0.027);
    position_tolerance_ = this->declare_parameter<double>("tolerance.position", 0.0005);
    max_correction_step_ = this->declare_parameter<double>("max_correction_step", 0.001);

    fjt_action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this, "/mg400_arm_controller/follow_joint_trajectory");
        
    sensor_x_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/distance/sensor_0", 10, std::bind(&PrecisionHomingNode::sensorXCallback, this, std::placeholders::_1));
    sensor_z_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/distance/sensor_1", 10, std::bind(&PrecisionHomingNode::sensorZCallback, this, std::placeholders::_1));
    
    homing_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/start_homing", std::bind(&PrecisionHomingNode::startHomingCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&PrecisionHomingNode::controlLoop, this));
    control_loop_timer_->cancel();

    control_streaming_client_ = rclcpp_action::create_client<hg_c1030_msgs::action::ControlStreaming>(
        this, "/control_streaming");

    RCLCPP_INFO(this->get_logger(), "Precision Homing Node is ready. Call the '/precision_homing_node/start_homing' service to begin.");
}

void PrecisionHomingNode::init()
{
    // It is now safe to call shared_from_this()
    const auto move_group_name = this->get_parameter("move_group_name").as_string();
    
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), 
        move_group_name
    );

    // You might also want to do a quick check to ensure it loaded
    if (move_group_->getPlanningFrame().empty()) {
        RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface failed to initialize.");
        // Handle error appropriately, maybe by throwing or shutting down
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Precision Homing Node is fully initialized and ready. Call the service to begin.");
}

void PrecisionHomingNode::setLaserStreamingState(bool power_on)
{
    using namespace std::placeholders;

    if (!control_streaming_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(this->get_logger(), "Control streaming action server not available!");
        if (pending_homing_response_)
        {
            pending_homing_response_->success = false;
            pending_homing_response_->message = "Action server for laser control not available.";
            pending_homing_response_ = nullptr; // Consume the response
        }
        return;
    }

    auto goal_msg = hg_c1030_msgs::action::ControlStreaming::Goal();
    goal_msg.start_streaming = power_on;

    auto send_goal_options = rclcpp_action::Client<hg_c1030_msgs::action::ControlStreaming>::SendGoalOptions();

    // Callback for when the server accepts/rejects the goal
    send_goal_options.goal_response_callback = [this](const rclcpp_action::ClientGoalHandle<hg_c1030_msgs::action::ControlStreaming>::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Laser control goal was rejected by server.");
            if (pending_homing_response_)
            {
                pending_homing_response_->success = false;
                pending_homing_response_->message = "Laser control goal was rejected.";
                pending_homing_response_ = nullptr;
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Laser control goal accepted by server, waiting for result...");
        }
    };

    // Callback for when the action finishes
    send_goal_options.result_callback = [this, power_on](const rclcpp_action::ClientGoalHandle<hg_c1030_msgs::action::ControlStreaming>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Successfully set laser power %s.", power_on ? "ON" : "OFF");

            // If we were turning it ON to start homing...
            if (power_on && pending_homing_response_)
            {
                RCLCPP_INFO(this->get_logger(), "Laser is ON. Starting homing control loop.");
                pid_x_.reset();
                pid_z_.reset();
                is_homing_active_ = true;
                control_loop_timer_->reset();
                pending_homing_response_->success = true;
                pending_homing_response_->message = "Precision homing started.";
                pending_homing_response_ = nullptr; // Consume the response
            }
            // If we were turning it OFF, we don't need to do anything else.
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Laser control goal was aborted.");
            if (pending_homing_response_)
            {
                pending_homing_response_->success = false;
                pending_homing_response_->message = "Laser control was aborted.";
                pending_homing_response_ = nullptr;
            }
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Laser control goal was canceled.");
            if (pending_homing_response_)
            {
                pending_homing_response_->success = false;
                pending_homing_response_->message = "Laser control was canceled.";
                pending_homing_response_ = nullptr;
            }
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code for laser control action.");
            if (pending_homing_response_)
            {
                pending_homing_response_->success = false;
                pending_homing_response_->message = "Unknown error in laser control.";
                pending_homing_response_ = nullptr;
            }
            return;
        }
    };

    control_streaming_client_->async_send_goal(goal_msg, send_goal_options);
}

void PrecisionHomingNode::startHomingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
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

    // Store the response object so the result_callback can fill it in later.
    pending_homing_response_ = response;

    // This call is now non-blocking. It will return immediately.
    setLaserStreamingState(true);
}

void PrecisionHomingNode::controlLoop()
{
    if (!is_homing_active_ || !move_group_) // Add a check here
        return;

    double error_x = target_distance_x_ - latest_distance_x_;
    double error_z = target_distance_z_ - latest_distance_z_;

    if (std::abs(error_x) < position_tolerance_ && std::abs(error_z) < position_tolerance_)
    {
        RCLCPP_INFO(this->get_logger(), "Homing successful! Position within tolerance.");
        is_homing_active_ = false;
        control_loop_timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "Turning off laser power after homing completion.");
        setLaserStreamingState(false); // Call the async function to turn it off.
        return;
    }

    // ... rest of the control loop is the same ...
    double correction_x = pid_x_.compute(error_x);
    double correction_z = pid_z_.compute(error_z);
    correction_x = std::clamp(correction_x, -max_correction_step_, max_correction_step_);
    correction_z = std::clamp(correction_z, -max_correction_step_, max_correction_step_);
    RCLCPP_DEBUG(this->get_logger(), "Errors (x,z): [%.4f, %.4f]. Corrections (x,z): [%.4f, %.4f]", error_x, error_z, correction_x, correction_z);
    auto current_pose = move_group_->getCurrentPose().pose;
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x += correction_x;
    target_pose.position.z += correction_z;
    move_group_->setPoseTarget(target_pose);
    std::vector<double> target_joints;
    move_group_->getJointValueTarget(target_joints);
    sendCorrectionCommand(target_joints);
}

// The sensor callbacks and sendCorrectionCommand function remain unchanged.
void PrecisionHomingNode::sensorXCallback(const sensor_msgs::msg::Range::SharedPtr msg) { latest_distance_x_ = msg->range; }
void PrecisionHomingNode::sensorZCallback(const sensor_msgs::msg::Range::SharedPtr msg) { latest_distance_z_ = msg->range; }

void PrecisionHomingNode::sendCorrectionCommand(const std::vector<double> &joint_positions)
{
    if (!fjt_action_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(this->get_logger(), "FollowJointTrajectory action server not available!");
        is_homing_active_ = false;
        control_loop_timer_->cancel();
        return;
    }
    control_msgs::action::FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory.joint_names = move_group_->getJointNames();
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_positions;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    goal_msg.trajectory.points.push_back(point);
    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    fjt_action_client_->async_send_goal(goal_msg, send_goal_options);
}

// main function remains unchanged.
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