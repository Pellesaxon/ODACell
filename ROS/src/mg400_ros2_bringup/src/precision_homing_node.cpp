
#include "mg400_ros2_bringup/precision_homing_node.hpp"
#include <algorithm>
#include <vector>

#define USE_SKRRRRR 0 // Set to 1 to use the skrrrrr method, 0 for PID

static double to_rad(double deg) { return deg * M_PI / 180.0; }
static double to_deg(double rad) { return rad * 180.0 / M_PI; }

// Constructor
PrecisionHomingNode::PrecisionHomingNode(const rclcpp::NodeOptions &options)
    : Node("precision_homing_node", options),
      pid_x_(
          this->declare_parameter<double>("pid.x.kp", 5.0), // 0.7
          this->declare_parameter<double>("pid.x.ki", 0.0), // 0.03
          this->declare_parameter<double>("pid.x.kd", 0.0), // 0.06
          this->declare_parameter<double>("pid.x.min_output", -to_rad(0.2)),
          this->declare_parameter<double>("pid.x.max_output", to_rad(0.2)),
          this->get_clock()),
      // PID for Y-axis error (side/side), which we will map to Joint 1
      pid_y_(
          this->declare_parameter<double>("pid.y.kp", 0.0), // 0.7
          this->declare_parameter<double>("pid.y.ki", 0.0), // 0.03
          this->declare_parameter<double>("pid.y.kd", 0.0), // 0.06
          this->declare_parameter<double>("pid.y.min_output", -to_rad(0.2)),
          this->declare_parameter<double>("pid.y.max_output", to_rad(0.2)),
          this->get_clock())
{
    RCLCPP_INFO(this->get_logger(), "Initializing Precision Homing Node (Direct Joint Control)...");

    current_distance_x_.store(-1.0);
    current_distance_y_.store(-1.0);
    has_received_joint_state_.store(false);
    current_joint_angles_.resize(4, 0.0);

    action_server_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // --- Initialize Subscriptions ---
    rclcpp::SubscriptionOptions sub_options;
    sensor_x_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/distance/sensor_0", 10, std::bind(&PrecisionHomingNode::sensorXCallback, this, std::placeholders::_1), sub_options);
    sensor_y_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/distance/sensor_1", 10, std::bind(&PrecisionHomingNode::sensorYCallback, this, std::placeholders::_1), sub_options);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&PrecisionHomingNode::jointStateCallback, this, std::placeholders::_1));

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
    if (!std::isinf(msg->range))
    {
        current_distance_x_.store(msg->range);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "X sensor range is infinite, resetting to -1.0");
        current_distance_x_.store(-1.0);
    }
}

void PrecisionHomingNode::sensorYCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    if (!std::isinf(msg->range))
    {
        current_distance_y_.store(msg->range);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "Y sensor range is infinite, resetting to -1.0");
        current_distance_y_.store(-1.0);
    }
}

void PrecisionHomingNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (msg->name.size() == 4)
    {

        auto it_j1 = std::find(msg->name.begin(), msg->name.end(), "j1");
        auto it_j2 = std::find(msg->name.begin(), msg->name.end(), "j2");
        auto it_j3 = std::find(msg->name.begin(), msg->name.end(), "j3");
        auto it_j4 = std::find(msg->name.begin(), msg->name.end(), "j4");

        if (it_j1 != msg->name.end() && it_j2 != msg->name.end() &&
            it_j3 != msg->name.end() && it_j4 != msg->name.end())
        {
            current_joint_angles_[0] = msg->position[std::distance(msg->name.begin(), it_j1)];
            current_joint_angles_[1] = msg->position[std::distance(msg->name.begin(), it_j2)];
            current_joint_angles_[2] = msg->position[std::distance(msg->name.begin(), it_j3)];
            current_joint_angles_[3] = msg->position[std::distance(msg->name.begin(), it_j4)];

            if (!has_received_joint_state_.load())
            {
                has_received_joint_state_.store(true);
            }
        }
    }
}

bool PrecisionHomingNode::setLaserStreamingState(bool power_on)
{
    if (!laser_control_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(this->get_logger(), "Laser control action server not available!");
        return false;
    }
    auto goal_msg = LaserControlAction::Goal();
    goal_msg.start_streaming = power_on;
    RCLCPP_INFO(this->get_logger(), "Requesting to %s laser streaming...", power_on ? "START" : "STOP");

    auto goal_handle_future = laser_control_client_->async_send_goal(goal_msg);

    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get goal handle from laser control server.");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Laser control goal was rejected by server.");
        return false;
    }

    // 2. Now wait for the result
    auto result_future = laser_control_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get result from laser control server.");
        return false;
    }

    auto result_wrapper = result_future.get();
    if (result_wrapper.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
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

void PrecisionHomingNode::execute_homing_pid(const std::shared_ptr<GoalHandleHoming> goal_handle)
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
    while (rclcpp::ok() && (!has_received_joint_state_.load() || current_distance_x_.load() < 0 || current_distance_y_.load() < 0))
    {
        if ((this->get_clock()->now() - start_wait).seconds() > 10.0)
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for initial sensor or joint state data.");
            setLaserStreamingState(false);
            result->success = false;
            result->message = "Timeout waiting for initial data.";
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
    RCLCPP_INFO(this->get_logger(), "Starting micro-move PID loop.");
    pid_x_.reset();
    pid_y_.reset();

    rclcpp::Rate loop_rate(5); // Slower loop rate is fine for synchronous control
    int consecutive_successes = 0;
    const int success_threshold = 5;
    auto loop_start_time = this->get_clock()->now();
    const double timeout_seconds = rclcpp::Duration(goal->timeout).seconds();

    // State variables for the loop
    std::vector<double> local_current_joints;
    std::vector<double> last_known_good_joint_angles;

    // Initialize to first received joint states
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        local_current_joints = current_joint_angles_;
        last_known_good_joint_angles = current_joint_angles_;
    }

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

        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            local_current_joints = current_joint_angles_;
        }

        double dist_x = current_distance_x_.load();
        double dist_y = current_distance_y_.load();

        RCLCPP_INFO(this->get_logger(), "Current distances: X=%.4f m, Y=%.4f m", dist_x, dist_y);

        // --- Out-of-Range Recovery Logic ---
        if (dist_x < 0 || dist_y < 0)
        {
            RCLCPP_WARN(this->get_logger(), "Sensor data out of range! Attempting to move back to last good position.");
            auto move_goal = MoveToJointAction::Goal();
            std::copy(last_known_good_joint_angles.begin(), last_known_good_joint_angles.end(), move_goal.joint_angles.begin());
            move_goal.speed_percent = 5.0f; // Use a slightly faster speed for recovery
            move_goal.acc_percent = 5.0f;

            RCLCPP_INFO(this->get_logger(), "Sending recovery command and waiting for completion...");

            auto goal_handle_future = move_to_joint_client_->async_send_goal(move_goal);
            if (goal_handle_future.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
            {
                auto goal_handle_move = goal_handle_future.get();
                if (goal_handle_move)
                {
                    auto result_future = move_to_joint_client_->async_get_result(goal_handle_move);
                    result_future.wait_for(std::chrono::seconds(10));
                }
            }

            RCLCPP_INFO(this->get_logger(), "Recovery move sent. Pausing to re-acquire sensors.");
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            pid_x_.reset(); // Reset PID integrals after a fault
            pid_y_.reset();
            continue; // Restart the main control loop
        }

        // If we are here, sensors are good, so update our recovery position
        last_known_good_joint_angles = current_joint_angles_;

        // Calculate error
        double error_x = goal->target_distance_x - dist_x;
        // The Y sensor is on the side, so its target distance might be different
        double error_y = goal->target_distance_y - dist_y;

        // Publish feedback
        feedback->current_distance_x = dist_x;
        feedback->current_distance_y = dist_y;
        feedback->error_x = error_x;
        feedback->error_y = error_y;
        goal_handle->publish_feedback(feedback);

        // Check for success condition
        if (std::abs(error_x) < goal->tolerance && std::abs(error_y) < goal->tolerance)
        {
            consecutive_successes++;
            if (consecutive_successes >= success_threshold)
            {
                RCLCPP_INFO(this->get_logger(), "Homing successful! Position stable.");
                break; // SUCCESS!
            }
            loop_rate.sleep();
            continue; // We are close but not stable yet, so wait.
        }
        else
        {
            consecutive_successes = 0;
        }

        // --- DIRECT JOINT MOVE EXECUTION ---

        // Calculate angular corrections. Signs must be validated experimentally!
        // Assumption: Positive X error (too far) needs positive J2 to move forward.
        // Assumption: Positive Y error (too far right) needs NEGATIVE J1 to rotate left.
        double dj2 = pid_x_.compute(error_x);
        double dj1 = pid_y_.compute(error_y);

        // Bound the maximum correction per step
        const double max_correction_rad = to_rad(1); // Max 0.5 degree move per step
        dj1 = std::clamp(dj1, -max_correction_rad, max_correction_rad);
        dj2 = std::clamp(dj2, -max_correction_rad, max_correction_rad);

        // Original sign directions => J1: -Y, J2: +X

        std::vector<double> target_joints = local_current_joints;
        target_joints[0] -= dj1;
        target_joints[1] += dj2;

        auto move_goal = MoveToJointAction::Goal();
        std::copy(target_joints.begin(), target_joints.end(), move_goal.joint_angles.begin());
        move_goal.speed_percent = 2.0f;
        move_goal.acc_percent = 2.0f;

        RCLCPP_INFO(this->get_logger(), "Correction: dJ1=%.4f deg, dJ2=%.4f deg", to_deg(dj1), to_deg(dj2));

        auto goal_handle_future = move_to_joint_client_->async_send_goal(move_goal);

        if (goal_handle_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready)
        {
            RCLCPP_ERROR(this->get_logger(), "MoveToJoint goal send timed out.");
            continue;
        }

        auto goal_handle_move = goal_handle_future.get();
        if (!goal_handle_move)
        {
            RCLCPP_ERROR(this->get_logger(), "MoveToJoint goal was rejected by the server.");
            continue;
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for micro-move to complete...");
        auto result_future = move_to_joint_client_->async_get_result(goal_handle_move);
        // Very generous timeout as we move slowly
        if (result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready)
        {
            RCLCPP_ERROR(this->get_logger(), "Timed out waiting for MoveToJoint action result. (>30 seconds)");
        }
        else
        {
            auto result_wrapper = result_future.get();
            if (result_wrapper.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Micro-move step completed.");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Micro-move step did not succeed.");
            }
        }
    }

    // --- Cleanup ---
    setLaserStreamingState(false);
    result->success = true;
    result->message = "Precision homing completed successfully.";
    goal_handle->succeed(result);
}

void PrecisionHomingNode::execute_homing_skrrrrr(const std::shared_ptr<GoalHandleHoming> goal_handle)
{
    // "Dumb" implementation of homing sequence.
    RCLCPP_INFO(this->get_logger(), "Executing precision homing using SKRRRRR method...");
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
    while (rclcpp::ok() && (!has_received_joint_state_.load() ||
                            current_distance_x_.load() < 0 || current_distance_y_.load() < 0))
    {
        if ((this->get_clock()->now() - start_wait).seconds() > 10.0)
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for initial sensor or joint state data.");
            setLaserStreamingState(false);
            result->success = false;
            result->message = "Timeout waiting for initial data.";
            goal_handle->abort(result);
            return;
        }
    }

    // If we reach here, it means we have valid sensor data
    RCLCPP_INFO(this->get_logger(), "Initial sensor data received.");

    // --- SKRRRRR CONTROL LOOP ---
    RCLCPP_INFO(this->get_logger(), "Starting SKRRRRR homing loop.");

    rclcpp::Time loop_start_time = this->get_clock()->now();
    const double timeout_seconds = rclcpp::Duration(goal->timeout).seconds();

    std::vector<double> local_current_joints;
    std::vector<double> last_known_good_joint_angles;

    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        local_current_joints = current_joint_angles_;
        last_known_good_joint_angles = current_joint_angles_;
    }

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
        double dist_y = current_distance_y_.load();

        RCLCPP_INFO(this->get_logger(), "Current distances: X=%.4f m, Y=%.4f m", dist_x, dist_y);
        break;
    }

    return;
}

void PrecisionHomingNode::execute_homing(const std::shared_ptr<GoalHandleHoming> goal_handle)
{
#ifdef USE_SKRRRRR
    execute_homing_skrrrrr(goal_handle);
#else
    execute_homing_pid(goal_handle);
#endif
    RCLCPP_INFO(this->get_logger(), "Homing execution thread finished.");
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