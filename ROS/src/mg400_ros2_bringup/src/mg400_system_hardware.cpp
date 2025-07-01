
/*
* @file mg400_system_hardware.cpp
* @brief Implementation file for the MG400SystemHardware class, which manages the hardware interface for the MG400 robot.
*
* @author LT
* @version 1.0
* @date 2025-06-25
*/

#include "mg400_ros2_bringup/mg400_system_hardware.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mg400_ros2_bringup
{

MG400SystemHardware::MG400SystemHardware() : shm_bridge_("DEFAULT_NO_NAME")
{   
}

hardware_interface::CallbackReturn MG400SystemHardware::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    if (hardware_interface::SystemInterface::on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("MG400SystemHardware"), "Configuring hardware for robot: '%s'", "mg400");
    shm_bridge_ = SharedMemoryBridge("mg400");

    hw_positions_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);
    hw_velocity_commands_.resize(info_.joints.size(), 0.0);
    hw_acceleration_commands_.resize(info_.joints.size(), 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MG400SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MG400SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_acceleration_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn MG400SystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("MG400SystemHardware"), "Activating.. attaching shared memory bridge");
    if (!shm_bridge_.attach())
    {
        RCLCPP_FATAL(rclcpp::get_logger("MG400SystemHardware"), "Failed to attach to shared memory! Is the MG400 driver node running?");
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < hw_positions_.size(); i++)
    {
        hw_positions_[i] = shm_bridge_.shared_memory_ptr->q_actual[i];
        hw_commands_[i] = hw_positions_[i]; // Initialize commands with current positions
        hw_velocity_commands_[i] = 0.0;
        hw_acceleration_commands_[i] = 0.0;
    }
    if (shm_bridge_.shared_memory_ptr) { shm_bridge_.shared_memory_ptr->new_command_flag = false; } // Reset command flag
    RCLCPP_INFO(rclcpp::get_logger("MG400SystemHardware"), "Successfully activated and attached to shared memory.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MG400SystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("MG400SystemHardware"), "Deactivating..");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MG400SystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (shm_bridge_.shared_memory_ptr)
    {
        for (size_t i = 0; i < hw_positions_.size(); i++)
        {
            hw_positions_[i] = shm_bridge_.shared_memory_ptr->q_actual[i];
            hw_velocities_[i] = shm_bridge_.shared_memory_ptr->qd_actual[i];
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MG400SystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (shm_bridge_.shared_memory_ptr)
    {
        for (size_t i = 0; i < hw_commands_.size(); i++)
        {
            shm_bridge_.shared_memory_ptr->q_command[i] = hw_commands_[i];
            shm_bridge_.shared_memory_ptr->qd_command[i] = hw_velocity_commands_[i];
            shm_bridge_.shared_memory_ptr->qdd_command[i] = hw_acceleration_commands_[i];
        }
        shm_bridge_.shared_memory_ptr->new_command_flag = true; // Set the command flag to indicate new commands are available
    }
    return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mg400_ros2_bringup::MG400SystemHardware, hardware_interface::SystemInterface)