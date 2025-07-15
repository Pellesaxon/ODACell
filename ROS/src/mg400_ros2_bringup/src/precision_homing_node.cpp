#include "mg400_ros2_bringup/precision_homing_node.hpp"
#include <algorithm> // For std::copy, std::clamp
#include <vector>

static double to_rad(double deg) { return deg * M_PI / 180.0; }
static double to_deg(double rad) { return rad * 180.0 / M_PI; }

// Constructor
PrecisionHomingNode::PrecisionHomingNode(const rclcpp::NodeOptions &options)
    : Node("precision_homing_node", options),
      pid_x_(
          this->declare_parameter<double>("pid.x.kp", 0.5),                  // Proportional gain
          this->declare_parameter<double>("pid.x.ki", 0.02),                 // Integral gain
          this->declare_parameter<double>("pid.x.kd", 0.05),                 // Derivative gain
          this->declare_parameter<double>("pid.x.min_output", -to_rad(0.1)), // Min correction in rad
          this->declare_parameter<double>("pid.x.max_output", to_rad(0.1)),  // Max correction in rad
          this->get_clock()),
      pid_z_(
          this->declare_parameter<double>("pid.z.kp", 0.5),
          this->declare_parameter<double>("pid.z.ki", 0.02),
          this->declare_parameter<double>("pid.z.kd", 0.05),
          this->declare_parameter<double>("pid.z.min_output", -to_rad(0.1)),
          this->declare_parameter<double>("pid.z.max_output", to_rad(0.1)),
          this->get_clock())
{
    RCLCPP_INFO(this->get_logger(), "Initializing Precision Homing Node (Direct Joint Control)...");

    current_distance_x_.store(-1.0);
    current_distance_z_.store(-1.0);

    action_server_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // --- Initialize Subscriptions ---
    rclcpp::SubscriptionOptions sub_options;
    sensor_x_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/distance/sensor_0", 10, std::bind(&PrecisionHomingNode::sensorXCallback, this, std::placeholders::_1), sub_options);
    sensor_z_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/distance/sensor_1", 10, std::bind(&PrecisionHomingNode::sensorZCallback, this, std::placeholders::_1), sub_options);

    // --- Initialize Action Clients & Servers ---
    laser_control_client_ = rclcpp_action::create_client<LaserControlAction>(this, "/control_streaming", client_cb_group_);

    move_to_joint_client_ = rclcpp_action::create_client<MoveToJointAction>(this, "mg400/move_to_joint", client_cb_group_);
    if (!move_to_joint_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_FATAL(this->get_logger(), "MoveToJoint action server not available! Is the driver running?");
        throw std::runtime_error("MoveToJoint action server not available.");
    }

    homing_action_server_ = rclcpp_action::create_server<HomingAction>(
        this, "precision_homing",
        std::bind(&PrecisionHomingNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PrecisionHomingNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&PrecisionHomingNode::handle_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(),
        action_server_cb_group_);

    RCLCPP_INFO(this->get_logger(), "Precision Homing Node is ready and waiting for goals.");
}

// Destructor
PrecisionHomingNode::~PrecisionHomingNode() {}

// --- Sensor Callbacks ---
void PrecisionHomingNode::sensorXCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    if (msg->range < msg->max_range && msg->range > msg->min_range)
    {
        current_distance_x_.store(msg->range);
    }
    else
    {
        current_distance_x_.store(-1.0);
    }
}

void PrecisionHomingNode::sensorZCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    if (msg->range < msg->max_range && msg->range > msg->min_range)
    {
        current_distance_z_.store(msg->range);
    }
    else
    {
        current_distance_z_.store(-1.0);
    }
}

bool PrecisionHomingNode::setLaserStreamingState(bool power_on)
{
    if (!laser_control_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Laser control action server not available!");
        return false;
    }
    auto goal_msg = LaserControlAction::Goal();
    goal_msg.start_streaming = power_on;
    RCLCPP_INFO(this->get_logger(), "Requesting to %s laser streaming...", power_on ? "START" : "STOP");

    auto goal_handle_future = laser_control_client_->async_send_goal(goal_msg);

    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get goal handle from laser control server.");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Laser control goal was rejected by server.");
        return false;
    }

    // 2. Now wait for the result
    auto result_future = laser_control_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get result from laser control server.");
        return false;
    }
    
    auto result_wrapper = result_future.get();
    if (result_wrapper.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Laser control action succeeded: %s", result_wrapper.result->message.c_str());
        return result_wrapper.result->success;
    }

    RCLCPP_ERROR(this->get_logger(), "Laser control action failed with code %d", static_cast<int>(result_wrapper.code));
    return false;
}

// --- Homing Action Server Handlers ---
rclcpp_action::GoalResponse PrecisionHomingNode::handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const HomingAction::Goal>)
{
    RCLCPP_INFO(this->get_logger(), "Received homing goal request. Accepting.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PrecisionHomingNode::handle_cancel(
    const std::shared_ptr<GoalHandleHoming>)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel homing goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PrecisionHomingNode::handle_accepted(
    const std::shared_ptr<GoalHandleHoming> goal_handle)
{
    std::thread{std::bind(&PrecisionHomingNode::execute_homing, this, std::placeholders::_1), goal_handle}.detach();
}

void PrecisionHomingNode::execute_homing(const std::shared_ptr<GoalHandleHoming> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing precision homing using DIRECT JOINT control...");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<HomingAction::Result>();
    auto feedback = std::make_shared<HomingAction::Feedback>();

    if (!setLaserStreamingState(true))
    {
        result->success = false;
        result->message = "Failed to turn on laser sensors.";
        goal_handle->abort(result);
        return;
    }

    rclcpp::Rate wait_rate(10);
    auto start_wait = this->get_clock()->now();
    while (rclcpp::ok() && (current_distance_x_.load() < 0 || current_distance_z_.load() < 0))
    {
        if ((this->get_clock()->now() - start_wait).seconds() > 5.0)
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for valid in-range sensor data.");
            setLaserStreamingState(false);
            result->success = false;
            result->message = "Timeout waiting for sensor data.";
            goal_handle->abort(result);
            return;
        }
        if (goal_handle->is_canceling())
        { // Check for cancel during wait
            goal_handle->canceled(result);
            return;
        }
        wait_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Initial sensor data received and is in valid range.");

    // --- CONTROL LOOP ---
    RCLCPP_INFO(this->get_logger(), "Starting micro-move (Direct Joint PID loop).");
    pid_x_.reset();
    pid_z_.reset();

    rclcpp::Rate loop_rate(5); // Loop at 5 Hz
    int consecutive_successes = 0;
    const int success_threshold = 5; // Needs ~0.5s of stability
    auto loop_start_time = this->get_clock()->now();
    const double timeout_seconds = rclcpp::Duration(goal->timeout).seconds();

    std::vector<double> current_joint_angles(4, 0.0);

    while (rclcpp::ok())
    {

        if (goal_handle->is_canceling())
        {
            RCLCPP_INFO(this->get_logger(), "Homing canceled by client.");
            setLaserStreamingState(false);
            result->success = false;
            result->message = "Homing canceled by client.";
            goal_handle->canceled(result);
            return;
        }
        if ((this->get_clock()->now() - loop_start_time).seconds() > timeout_seconds)
        {
            RCLCPP_ERROR(this->get_logger(), "Homing timed out.");
            setLaserStreamingState(false);
            result->success = false;
            result->message = "Homing timed out.";
            goal_handle->abort(result);
            return;
        }

        double dist_x = current_distance_x_.load();
        double dist_z = current_distance_z_.load();

        if (dist_x < 0 || dist_z < 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Sensor data is out of range, pausing PID.");
            loop_rate.sleep();
            continue;
        }

        double error_x = goal->target_distance_x - dist_x;
        double error_z = goal->target_distance_z - dist_z;

        feedback->current_distance_x = dist_x;
        feedback->current_distance_z = dist_z;
        feedback->error_x = error_x;
        feedback->error_z = error_z;
        goal_handle->publish_feedback(feedback);

        if (std::abs(error_x) < goal->tolerance && std::abs(error_z) < goal->tolerance)
        {
            consecutive_successes++;
            if (consecutive_successes >= success_threshold)
            {
                RCLCPP_INFO(this->get_logger(), "Homing successful! Position stable within tolerance.");
                break; 
            }
            loop_rate.sleep();
            continue; // close, just wait for stability
        }
        else
        {
            consecutive_successes = 0;
        }

        // --- DIRECT JOINT MOVE EXECUTION ---

        // Assumption: Positive X error (too far) needs positive J2 angle to move forward.
        // Assumption: Positive Z error (too high) needs positive J3 angle to move "down".
        double dj2 = pid_x_.compute(error_x);
        double dj3 = pid_z_.compute(error_z);

        // 2. Define the target joint angles based on the last known position
        std::vector<double> target_joints = current_joint_angles;
        target_joints[1] += dj2; // Apply correction to J2
        target_joints[2] += dj3; // Apply correction to J3
        // Keep J1 and J4 constant
        target_joints[0] = current_joint_angles[0];
        target_joints[3] = current_joint_angles[3];

        auto move_goal = MoveToJointAction::Goal();
        std::copy(target_joints.begin(), target_joints.end(), move_goal.joint_angles.begin());
        move_goal.speed_percent = 2.0f; // Use very slow moves
        move_goal.acc_percent = 2.0f;

        RCLCPP_INFO(this->get_logger(), "Sending correction: dJ2=%.4f deg, dJ3=%.4f deg", to_deg(dj2), to_deg(dj3));

        auto goal_handle_future = move_to_joint_client_->async_send_goal(move_goal);
        if (goal_handle_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "MoveToJoint goal send timed out.");
            continue; // Skip to next loop iteration
        }
        
        auto goal_handle_move = goal_handle_future.get();
        if (!goal_handle_move) {
            RCLCPP_ERROR(this->get_logger(), "MoveToJoint goal was rejected by the server.");
            continue; // Skip to next loop iteration
        }

        //Wait for the action to complete
        RCLCPP_INFO(this->get_logger(), "Waiting for micro-move to complete...");
        auto result_future = move_to_joint_client_->async_get_result(goal_handle_move);
        if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Timed out waiting for MoveToJoint action result.");
        } else {
            auto result_wrapper = result_future.get();
            if (result_wrapper.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Micro-move step completed.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Micro-move step failed, was canceled, or aborted. Code: %d", static_cast<int>(result_wrapper.code));
            }

        }
    }

    // --- Cleanup ---
    setLaserStreamingState(false);
    result->success = true;
    result->message = "Precision homing completed successfully.";
    goal_handle->succeed(result);
}

// --- Main function ---
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PrecisionHomingNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Spinning precision_homing_node with MultiThreadedExecutor.");
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}