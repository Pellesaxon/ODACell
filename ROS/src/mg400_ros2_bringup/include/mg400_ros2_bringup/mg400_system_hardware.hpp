
#ifndef MG400_SYSTEM_HARDWARE_HPP
#define MG400_SYSTEM_HARDWARE_HPP

/**
 * @file mg400_system_hardware.hpp
 * @brief Header file for the MG400SystemHardware class, which manages the hardware interface for the MG400 robot.
 *
 * This class provides functionality to initialize the hardware interface, read and write commands,
 * and manage the real-time state of the robot using shared memory.
 *
 * @version 1.0
 * @date 2025-06-25
 * @author LT 
 */

// MG400 project includes
#include "shared_memory_bridge.hpp"

// ROS2 includes
#include "rclcpp/macros.hpp"
#include "hardware_interface/system_interface.hpp"

namespace mg400_ros2_bringup
{
class MG400SystemHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MG400SystemHardware)

    MG400SystemHardware();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    SharedMemoryBridge shm_bridge_;
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_velocity_commands_;
    std::vector<double> hw_acceleration_commands_;

};

}

#endif // MG400_SYSTEM_HARDWARE_HPP