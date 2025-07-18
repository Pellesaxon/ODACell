// src/moveit_benchmark.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "mg400_msgs/action/benchmark.hpp"
//#include <moveit/planning_scene_interface/planning_scene_interface.hpp> // TODO: check if needed

#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <iomanip>
#include <atomic>
#include <fstream>
#include <ctime>

using namespace std::chrono_literals;

// Structs and helper classes remain the same...

// A struct to hold the results for each run
struct BenchmarkResult
{
    double speed_scale;
    double time_to_laser_test;
    double time_to_home;
    float sensor_0_distance;
    float sensor_1_distance;
};

// Node to subscribe to sensor topics and store the latest data
class SensorSubscriber : public rclcpp::Node
{
public:
    SensorSubscriber()
        : Node("sensor_subscriber_cpp")
    {
        sensor_0_dist_.store(-1.0f);
        sensor_1_dist_.store(-1.0f);

        subscription_0_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/distance/sensor_0", 10,
            [this](const sensor_msgs::msg::Range::SharedPtr msg)
            {
                sensor_0_dist_.store(msg->range);
            });

        subscription_1_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/distance/sensor_1", 10,
            [this](const sensor_msgs::msg::Range::SharedPtr msg)
            {
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


// Helper: generate a timestamped filename for the CSV
std::string generate_csv_filename()
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << "benchmark_results_" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".csv";
    return oss.str();
}

// Helper: write a single result to the CSV file
void write_result_to_csv(std::ofstream &file, const BenchmarkResult &result)
{
    file << std::fixed << std::setprecision(4)
         << result.speed_scale << ","
         << result.time_to_laser_test << ","
         << result.time_to_home << ","
         << result.sensor_0_distance << ","
         << result.sensor_1_distance << "\n";
}


// Helper: print final summary results to the console
void print_summary(const std::vector<BenchmarkResult> &results, const std::string& filename)
{
    std::cout << "\n--- BENCHMARKING COMPLETE ---\n";
    std::cout << "Results saved to: " << filename << "\n";
    std::cout << std::string(68, '=') << "\n";
    std::cout << "FINAL SUMMARY (" << results.size() << " runs completed)\n";
    std::cout << std::string(68, '=') << "\n";
    std::cout << std::left << std::setw(15) << "Speed Scale"
              << " | " << std::setw(20) << "Time to Laser (s)"
              << " | " << std::setw(20) << "Time to Home (s)\n";
    std::cout << std::string(68, '-') << "\n";

    for (const auto &r : results)
    {
        std::cout << std::fixed << std::setprecision(2)
                  << std::left << std::setw(15) << r.speed_scale << " | "
                  << std::setw(20) << r.time_to_laser_test << " | "
                  << std::setw(20) << r.time_to_home << "\n";
    }

    std::cout << "\n"
              << std::string(68, '=') << "\n";
    std::cout << std::left << std::setw(15) << "Speed Scale"
              << " | " << std::setw(20) << "Sensor 0 Dist (m)"
              << " | " << std::setw(20) << "Sensor 1 Dist (m)\n";
    std::cout << std::string(68, '-') << "\n";

    for (const auto &r : results)
    {
        std::cout << std::fixed << std::setprecision(4)
                  << std::left << std::setw(15) << r.speed_scale << " | ";

        if (r.sensor_0_distance < 0)
            std::cout << std::left << std::setw(20) << "N/A";
        else
            std::cout << std::left << std::setw(20) << r.sensor_0_distance;

        std::cout << " | ";

        if (r.sensor_1_distance < 0)
            std::cout << std::left << std::setw(20) << "N/A\n";
        else
            std::cout << std::left << std::setw(20) << r.sensor_1_distance << "\n";
    }

    std::cout << std::string(68, '=') << "\n";
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    double duration_minutes = 10.0;
    if (argc > 1) {
        try {
            duration_minutes = std::stod(argv[1]);
        } catch (const std::exception &e) {
            std::cerr << "Error: Invalid duration provided. Please use a number. Using default "
                      << duration_minutes << " minutes." << std::endl;
        }
    }
    const auto benchmark_duration = std::chrono::duration<double>(duration_minutes * 60.0);

    auto moveit_node = std::make_shared<rclcpp::Node>("moveit_benchmark_cpp");
    auto sensor_node = std::make_shared<SensorSubscriber>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(moveit_node);
    executor.add_node(sensor_node);
    std::thread exec_thread([&executor]() { executor.spin(); });

    // --- FIX: Use MoveGroupInterface as per documentation ---
    const std::string PLANNING_GROUP = "mg400_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(moveit_node, PLANNING_GROUP);

    auto logger = moveit_node->get_logger();
    RCLCPP_INFO(logger, "Starting movement benchmark for %.1f minutes...", duration_minutes);

    std::string csv_filename = generate_csv_filename();
    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
        RCLCPP_FATAL(logger, "Failed to open results file: %s", csv_filename.c_str());
        rclcpp::shutdown();
        return 1;
    }
    csv_file << "SpeedScale,TimeToLaser,TimeToHome,Sensor0_Dist,Sensor1_Dist\n";

    std::vector<BenchmarkResult> results;
    int run_count = 0;
    const auto start_time = std::chrono::steady_clock::now();

    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start_time < benchmark_duration))
    {
        double speed = ((run_count % 10) + 1) / 10.0;
        RCLCPP_INFO(logger, "Run %d: Testing with velocity scale: %.1f", run_count + 1, speed);

        // --- FIX: Set scaling factors directly on the MoveGroupInterface ---
        move_group_interface.setMaxVelocityScalingFactor(speed);
        move_group_interface.setMaxAccelerationScalingFactor(speed);

        BenchmarkResult r;
        r.speed_scale = speed;

        // --- FIX: Define a Plan object to store the trajectory ---
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success;

        // 1) Go home (using the simpler 'move' command for setup)
        move_group_interface.setNamedTarget("home");
        if (move_group_interface.move() != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(logger, "Home move failed; skipping speed %.1f", speed);
            std::this_thread::sleep_for(1s);
            continue;
        }
        std::this_thread::sleep_for(1s);

        // 2) Laser test (separating plan and execute for timing)
        move_group_interface.setNamedTarget("laser_test");
        success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        auto t0 = std::chrono::steady_clock::now();
        if (success && (move_group_interface.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            auto t1 = std::chrono::steady_clock::now();
            r.time_to_laser_test = std::chrono::duration<double>(t1 - t0).count();
            RCLCPP_INFO(logger, "Laser_test: %.2fs", r.time_to_laser_test);
        }
        else
        {
            RCLCPP_ERROR(logger, "Laser_test failed");
            r.time_to_laser_test = -1.0;
            continue;
        }

        // 3) Sample sensors
        std::this_thread::sleep_for(5s);
        sensor_node->get_distances(r.sensor_0_distance, r.sensor_1_distance);
        RCLCPP_INFO(logger, "Sensors: 0=%.3fm, 1=%.3fm", r.sensor_0_distance, r.sensor_1_distance);

        // 4) Back home (separating plan and execute for timing)
        move_group_interface.setNamedTarget("home");
        success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        t0 = std::chrono::steady_clock::now();
        if (success && (move_group_interface.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            auto t2 = std::chrono::steady_clock::now();
            r.time_to_home = std::chrono::duration<double>(t2 - t0).count();
            RCLCPP_INFO(logger, "Back home: %.2fs", r.time_to_home);
        }
        else
        {
            RCLCPP_ERROR(logger, "Return home failed");
            r.time_to_home = -1.0;
        }
        
        results.push_back(r);
        write_result_to_csv(csv_file, r);
        
        run_count++;
        std::this_thread::sleep_for(1s);
    }

    // Print & shutdown
    csv_file.close();
    print_summary(results, csv_filename);
    executor.cancel();
    exec_thread.join();
    rclcpp::shutdown();
    return 0;
}