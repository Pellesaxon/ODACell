#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <iomanip>
#include <atomic>
#include <fstream>
#include <ctime>
#include <sstream>

#include "mg400_msgs/action/benchmark.hpp"
#include "hg_c1030_msgs/action/control_streaming.hpp"

using namespace std::chrono_literals;

// A struct to hold the results for each run
struct BenchmarkResult
{
    double speed_scale;
    float sensor_0_distance;
    float sensor_1_distance;
    std::string start_position;
};

class BenchmarkActionServer : public rclcpp::Node
{
private:
    std::string generate_csv_filename()
    {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << "benchmark_results_" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".csv";
        return oss.str();
    }

    void write_result_to_csv(std::ofstream &file, const BenchmarkResult &result)
    {
        file << std::fixed << std::setprecision(6)
             << result.speed_scale << ","
             << result.start_position << ","
             << result.sensor_0_distance << ","
             << result.sensor_1_distance << "\n" << std::flush;
    }

public:
    using Benchmark = mg400_msgs::action::Benchmark;
    using GoalHandleBenchmark = rclcpp_action::ServerGoalHandle<Benchmark>;
    
    using LaserControlAction = hg_c1030_msgs::action::ControlStreaming;
    using LaserControlClient = rclcpp_action::Client<LaserControlAction>;

    explicit BenchmarkActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("benchmark_action_server", options), goal_active_(false), is_moveit_ready_(false)
    {
        // --- Setup that does NOT depend on shared_from_this() ---

        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Set up the benchmark action server
        this->action_server_ = rclcpp_action::create_server<Benchmark>(
            this, "benchmark",
            std::bind(&BenchmarkActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BenchmarkActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&BenchmarkActionServer::handle_accepted, this, std::placeholders::_1));

        // Set up action client for laser control
        this->laser_control_client_ = rclcpp_action::create_client<LaserControlAction>(
            this, "/control_streaming", client_cb_group_);

        // Set up sensor subscriptions
        sensor_0_dist_.store(-1.0f);
        sensor_1_dist_.store(-1.0f);

        subscription_0_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/distance/sensor_0", 10,
            [this](const sensor_msgs::msg::Range::SharedPtr msg) { sensor_0_dist_.store(msg->range); });

        subscription_1_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/distance/sensor_1", 10,
            [this](const sensor_msgs::msg::Range::SharedPtr msg) { sensor_1_dist_.store(msg->range); });

        // --- NEW: Defer MoveIt initialization using a one-shot timer ---
        // This timer will fire after the constructor is complete and the node is spinning.
        setup_timer_ = this->create_wall_timer(100ms, [this]() {
            this->setup_timer_->cancel(); // Ensure it only runs once
            this->setup_moveit();
        });

        RCLCPP_INFO(this->get_logger(), "Benchmark Action Server constructed. Waiting for MoveIt setup...");
    }

private:
    // --- NEW: Method for deferred initialization ---
    void setup_moveit()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface...");
        const std::string PLANNING_GROUP = "mg400_arm";
        // Now it's safe to call shared_from_this()
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
        
        move_group_interface_->setNumPlanningAttempts(10);
        move_group_interface_->setPlanningTime(10.0);
        
        is_moveit_ready_.store(true);
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface is initialized. Benchmark server is fully ready.");
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Benchmark::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received benchmark goal request for %.2f minutes", goal->duration_minutes);
        (void)uuid;

        // --- NEW: Readiness check ---
        if (!is_moveit_ready_.load()) {
            RCLCPP_ERROR(this->get_logger(), "Server is not ready, MoveIt is still initializing. Rejecting goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->duration_minutes <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Duration must be positive. Rejecting goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal_active_.load()) {
            RCLCPP_WARN(this->get_logger(), "Benchmark already in progress. Rejecting new goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleBenchmark> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel benchmark");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleBenchmark> goal_handle)
    {
        goal_active_.store(true);
        std::thread{std::bind(&BenchmarkActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }
    
    bool setLaserStreamingState(bool power_on)
    {
        if (!laser_control_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(get_logger(), "Laser control action server not available!");
            return false;
        }
        auto goal_msg = LaserControlAction::Goal();
        goal_msg.start_streaming = power_on;
        RCLCPP_INFO(get_logger(), "Requesting to %s laser streaming...", power_on ? "START" : "STOP");

        auto goal_handle_future = laser_control_client_->async_send_goal(goal_msg);
        if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
        {
            RCLCPP_ERROR(get_logger(), "Failed to get goal handle from laser control server.");
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Laser control goal was rejected by server.");
            return false;
        }

        auto result_future = laser_control_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
        {
            RCLCPP_ERROR(get_logger(), "Failed to get result from laser control server.");
            return false;
        }

        auto result_wrapper = result_future.get();
        if (result_wrapper.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(get_logger(), "Laser control action succeeded: %s", result_wrapper.result->message.c_str());
            return result_wrapper.result->success;
        }

        RCLCPP_ERROR(get_logger(), "Laser control action failed with code %d", static_cast<int>(result_wrapper.code));
        return false;
    }

    void execute(const std::shared_ptr<GoalHandleBenchmark> goal_handle); // Declaration only, definition below

    rclcpp_action::Server<Benchmark>::SharedPtr action_server_;
    LaserControlClient::SharedPtr laser_control_client_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_; 
    std::atomic<bool> goal_active_;

    // --- Member variables for MoveIt and Sensors ---
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_0_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_1_;
    std::atomic<float> sensor_0_dist_;
    std::atomic<float> sensor_1_dist_;
    
    // --- NEW: Timer and readiness flag ---
    rclcpp::TimerBase::SharedPtr setup_timer_;
    std::atomic<bool> is_moveit_ready_;
};

// --- Definition of execute method outside the class for clarity ---
void BenchmarkActionServer::execute(const std::shared_ptr<GoalHandleBenchmark> goal_handle)
{
    auto logger = get_logger();
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Benchmark::Feedback>();
    auto result = std::make_shared<Benchmark::Result>();
    
    std::string csv_filename = generate_csv_filename();
    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open())
    {
        RCLCPP_ERROR(logger, "Failed to open results file: %s", csv_filename.c_str());
        goal_handle->abort(result);
        goal_active_.store(false);
        return;
    }
    csv_file << "SpeedAndAccScale,StartPosition,Sensor0_Dist,Sensor1_Dist\n";
    RCLCPP_INFO(logger, "Logging results to %s", csv_filename.c_str());

    if (!setLaserStreamingState(true))
    {
        RCLCPP_ERROR(logger, "Failed to turn on lasers. Aborting benchmark.");
        result->total_runs = 0;
        result->results_filepath = csv_filename;
        goal_handle->abort(result);
        goal_active_.store(false);
        csv_file.close();
        return;
    }

    const auto benchmark_duration = std::chrono::duration<double>(goal->duration_minutes * 60.0);
    const auto start_time = std::chrono::steady_clock::now();
    int run_count = 0;
    bool was_cancelled = false;
    const std::vector<std::string> arm_start_positions = {"right_test_start", "left_test_start", "max_right_test", "max_left_test", "above_sensor_test"};

    

    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start_time < benchmark_duration))
    {
        if (goal_handle->is_canceling()) {
            was_cancelled = true;
            break;
        }

        double speed = ((run_count % 10) + 1) / 10.0;
        std::string start_position = arm_start_positions[run_count % arm_start_positions.size()];
        
        BenchmarkResult r;
        r.speed_scale = speed;
        r.start_position = start_position;
        r.sensor_0_distance = -1.0;
        r.sensor_1_distance = -1.0;

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        move_group_interface_->setMaxVelocityScalingFactor(speed);
        move_group_interface_->setMaxAccelerationScalingFactor(speed);
        move_group_interface_->setNamedTarget(start_position);
        if (move_group_interface_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger, "Move to start position '%s' failed; skipping run.", start_position.c_str());
            std::this_thread::sleep_for(1s);
            continue;
        }
        std::this_thread::sleep_for(500ms);

        move_group_interface_->setNamedTarget("laser_test");
        if (move_group_interface_->plan(my_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger, "Failed to plan laser test move; skipping run.");
            continue;
        }
        
        if (move_group_interface_->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(logger, "Failed to execute laser test move; skipping run.");
            continue;
        }

        std::this_thread::sleep_for(5s);
        
        r.sensor_0_distance = sensor_0_dist_.load();
        r.sensor_1_distance = sensor_1_dist_.load();

        if (r.sensor_0_distance < 0 || r.sensor_1_distance < 0) {
            RCLCPP_WARN(logger, "Failed to read valid sensor distances; skipping run.");
            continue;
        }

        std::stringstream status_stream;
        status_stream << "Run " << run_count + 1
                      << ": Speed=" << std::fixed << std::setprecision(1) << speed
                      << ", Sensor0=" << std::setprecision(6) << r.sensor_0_distance
                      << ", Sensor1=" << std::setprecision(6) << r.sensor_1_distance;
        feedback->runs_completed = run_count;
        feedback->status = status_stream.str();
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(logger, "%s", feedback->status.c_str());
        
        write_result_to_csv(csv_file, r);
        run_count++;
        std::this_thread::sleep_for(3s);
    }

    RCLCPP_INFO(logger, "Benchmark loop finished. Turning off lasers.");
    if (!setLaserStreamingState(false)) {
        RCLCPP_WARN(logger, "Failed to turn off lasers. Please check manually.");
    }

    RCLCPP_INFO(logger, "Returning home..");
    move_group_interface_->setNamedTarget("home");
    move_group_interface_->setMaxVelocityScalingFactor(1.0);
    move_group_interface_->setMaxAccelerationScalingFactor(1.0);
    if (move_group_interface_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Failed to return home after benchmark.");
    }

    result->total_runs = run_count;
    result->results_filepath = csv_filename;

    if (was_cancelled) {
        goal_handle->canceled(result);
        RCLCPP_INFO(logger, "Benchmark canceled by client after %d runs.", run_count);
    } else if (rclcpp::ok()) {
        goal_handle->succeed(result);
        RCLCPP_INFO(logger, "Benchmark finished successfully. Completed %d runs. Results in %s", run_count, csv_filename.c_str());
    } else {
        goal_handle->abort(result);
        RCLCPP_ERROR(logger, "Benchmark aborted due to ROS shutdown after %d runs.", run_count);
    }
    
    goal_active_.store(false);
    csv_file.close();
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<BenchmarkActionServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_server);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}