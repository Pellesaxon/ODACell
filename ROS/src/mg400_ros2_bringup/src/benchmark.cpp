// src/benchmark.cpp

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
    double time_to_laser_test;
    double time_to_home;
    float sensor_0_distance;
    float sensor_1_distance;
};

class BenchmarkActionServer : public rclcpp::Node
{
private:
    class SensorSubscriber : public rclcpp::Node
    {
    public:
        SensorSubscriber() : Node("sensor_subscriber_node")
        {
            sensor_0_dist_.store(-1.0f);
            sensor_1_dist_.store(-1.0f);

            subscription_0_ = this->create_subscription<sensor_msgs::msg::Range>(
                "/distance/sensor_0", 10,
                [this](const sensor_msgs::msg::Range::SharedPtr msg) {
                    sensor_0_dist_.store(msg->range);
                });

            subscription_1_ = this->create_subscription<sensor_msgs::msg::Range>(
                "/distance/sensor_1", 10,
                [this](const sensor_msgs::msg::Range::SharedPtr msg) {
                    sensor_1_dist_.store(msg->range);
                });
        }

        void get_distances(float &dist0, float &dist1)
        {
            dist0 = sensor_0_dist_.load();
            dist1 = sensor_1_dist_.load();
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_0_;
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_1_;
        std::atomic<float> sensor_0_dist_;
        std::atomic<float> sensor_1_dist_;
    };

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
             << result.time_to_laser_test << ","
             << result.time_to_home << ","
             << result.sensor_0_distance << ","
             << result.sensor_1_distance << "\n" << std::flush; // Flush to ensure data is written immediately
    }

public:

    using Benchmark = mg400_msgs::action::Benchmark;
    using GoalHandleBenchmark = rclcpp_action::ServerGoalHandle<Benchmark>;
    
    // Using declarations for the laser control action client
    using LaserControlAction = hg_c1030_msgs::action::ControlStreaming;
    using LaserControlClient = rclcpp_action::Client<LaserControlAction>;


    explicit BenchmarkActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("benchmark_action_server", options), goal_active_(false)
    {
        // Set up the benchmark action server
        this->action_server_ = rclcpp_action::create_server<Benchmark>(
            this,
            "benchmark",
            std::bind(&BenchmarkActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BenchmarkActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&BenchmarkActionServer::handle_accepted, this, std::placeholders::_1));

        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        
        this->laser_control_client_ = rclcpp_action::create_client<LaserControlAction>(
            this, 
            "/control_streaming", 
            client_cb_group_);

        RCLCPP_INFO(this->get_logger(), "Benchmark Action Server is ready.");
    }

private:
    rclcpp_action::Server<Benchmark>::SharedPtr action_server_;
    LaserControlClient::SharedPtr laser_control_client_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_; 
    std::atomic<bool> goal_active_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Benchmark::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received benchmark goal request for %.2f minutes", goal->duration_minutes);
        (void)uuid;

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

    
    void execute(const std::shared_ptr<GoalHandleBenchmark> goal_handle)
    {
        auto logger = get_logger();
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Benchmark::Feedback>();
        auto result = std::make_shared<Benchmark::Result>();
        
        auto sensor_node = std::make_shared<SensorSubscriber>();
        rclcpp::executors::SingleThreadedExecutor sensor_executor;
        sensor_executor.add_node(sensor_node);
        std::thread sensor_thread([&sensor_executor](){ sensor_executor.spin(); });

        const std::string PLANNING_GROUP = "mg400_arm";
        moveit::planning_interface::MoveGroupInterface move_group_interface(shared_from_this(), PLANNING_GROUP);
        
        std::string csv_filename = generate_csv_filename();
        std::ofstream csv_file(csv_filename);
        if (!csv_file.is_open())
        {
            RCLCPP_ERROR(logger, "Failed to open results file: %s", csv_filename.c_str());
            goal_handle->abort(result);
            goal_active_.store(false);
            sensor_executor.cancel();
            if(sensor_thread.joinable()) sensor_thread.join();
            return;
        }
        csv_file << "SpeedScale,TimeToLaser,TimeToHome,Sensor0_Dist,Sensor1_Dist\n";
        RCLCPP_INFO(logger, "Logging results to %s", csv_filename.c_str());

        // --- Turn on lasers before starting ---
        if (!setLaserStreamingState(true))
        {
            RCLCPP_ERROR(logger, "Failed to turn on lasers. Aborting benchmark.");
            result->total_runs = 0;
            result->results_filepath = csv_filename;
            goal_handle->abort(result);
            goal_active_.store(false);
            csv_file.close();
            sensor_executor.cancel();
            if(sensor_thread.joinable()) sensor_thread.join();
            return;
        }

        // --- Main Benchmark Loop ---
        const auto benchmark_duration = std::chrono::duration<double>(goal->duration_minutes * 60.0);
        const auto start_time = std::chrono::steady_clock::now();
        int run_count = 0;
        bool was_cancelled = false;

        double prev_distance_x = -1.0;
        double prev_distance_y = -1.0;
        double accumulative_error_x = 0.0;
        double accumulative_error_y = 0.0;

        while (rclcpp::ok() && (std::chrono::steady_clock::now() - start_time < benchmark_duration))
        {
            if (goal_handle->is_canceling()) {
                was_cancelled = true;
                break;
            }

            double speed = ((run_count % 10) + 1) / 10.0;

            move_group_interface.setMaxVelocityScalingFactor(speed);
            move_group_interface.setMaxAccelerationScalingFactor(speed);
            
            BenchmarkResult r;
            r.speed_scale = speed;
            r.time_to_laser_test = -1.0;
            r.time_to_home = -1.0;
            r.sensor_0_distance = -1.0;
            r.sensor_1_distance = -1.0;

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success;

            // 1) Go home
            move_group_interface.setNamedTarget("home");
            if (move_group_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(logger, "Initial home move failed; skipping run.");
                std::this_thread::sleep_for(1s);
                continue;
            }
            std::this_thread::sleep_for(500ms);

            // 2) Laser test
            move_group_interface.setNamedTarget("laser_test");
            success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            auto t0 = std::chrono::steady_clock::now();
            if (success && (move_group_interface.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)) {
                auto t1 = std::chrono::steady_clock::now();
                r.time_to_laser_test = std::chrono::duration<double>(t1 - t0).count();
            } else {
                RCLCPP_ERROR(logger, "Laser_test move failed; skipping run.");
                continue;
            }

            // 3) Sample sensors
            std::this_thread::sleep_for(5s); // Wait for robot to settle
            sensor_node->get_distances(r.sensor_0_distance, r.sensor_1_distance);
            if (r.sensor_0_distance < 0 || r.sensor_1_distance < 0) {
                RCLCPP_ERROR(logger, "Failed to read sensor distances; skipping run.");
                continue;
            }
            RCLCPP_INFO(logger, "Sensor 0: %.2f, Sensor 1: %.2f", r.sensor_0_distance, r.sensor_1_distance);
            if (prev_distance_x >= 0 && prev_distance_y >= 0) {
                accumulative_error_x += std::abs(r.sensor_0_distance - prev_distance_x);
                accumulative_error_y += std::abs(r.sensor_1_distance - prev_distance_y);
            }
            prev_distance_x = r.sensor_0_distance;
            prev_distance_y = r.sensor_1_distance;

            std::stringstream status_stream;
            status_stream << "Run " << run_count + 1
                          << ": Speed=" << std::fixed << std::setprecision(1) << speed
                          << ", Sensor0=" << std::setprecision(2) << r.sensor_0_distance
                          << ", Sensor1=" << std::setprecision(2) << r.sensor_1_distance
                          << ", AccumErr0=" << std::setprecision(4) << accumulative_error_x
                          << ", AccumErr1=" << std::setprecision(4) << accumulative_error_y;
            feedback->runs_completed = run_count;
            feedback->status = status_stream.str();
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(logger, "%s", feedback->status.c_str());
            
            // 4) Back home
            move_group_interface.setNamedTarget("home");
            success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            t0 = std::chrono::steady_clock::now();
            if (success && (move_group_interface.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)) {
                auto t2 = std::chrono::steady_clock::now();
                r.time_to_home = std::chrono::duration<double>(t2 - t0).count();
            } else {
                RCLCPP_ERROR(logger, "Return home move failed.");
            }
            
            write_result_to_csv(csv_file, r);
            run_count++;
            std::this_thread::sleep_for(1s);
        }

        RCLCPP_INFO(logger, "Benchmark loop finished. Turning off lasers.");
        if (!setLaserStreamingState(false)) {
            RCLCPP_WARN(logger, "Failed to turn off lasers. Please check manually.");
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
        sensor_executor.cancel();
        if(sensor_thread.joinable()) {
            sensor_thread.join();
        }
    }
};

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