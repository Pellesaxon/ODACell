#include "mg400_ros2_bringup/fast_start_slow_end_adapter.hpp"

#include <rclcpp/logging.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/robot_model.hpp>

namespace mg400_ros2_bringup
{
FastStartSlowEndAdapter::FastStartSlowEndAdapter() : logger_(rclcpp::get_logger("moveit.ros.fast_start_slow_end_adapter"))
{
}

std::string FastStartSlowEndAdapter::getDescription() const
{
  return "FastStartSlowEndAdapter";
}

void FastStartSlowEndAdapter::adapt(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
                                    const planning_interface::MotionPlanRequest& req,
                                    planning_interface::MotionPlanResponse& res) const
{
  RCLCPP_DEBUG(logger_, "Running '%s'", getDescription().c_str());
  if (!res.trajectory)
  {
    RCLCPP_ERROR(logger_,
                 "Cannot apply response adapter '%s' because MotionPlanResponse does not contain a path to scale.",
                 getDescription().c_str());
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return;
  }

  if (applyFastStartSlowEnd(req, *res.trajectory))
  {
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
  else
  {
    RCLCPP_ERROR(logger_, "Response adapter '%s' failed to generate a trajectory.", getDescription().c_str());
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }
}

bool FastStartSlowEndAdapter::applyFastStartSlowEnd(const planning_interface::MotionPlanRequest& req,
                                                    robot_trajectory::RobotTrajectory& trajectory) const
{
  if (trajectory.empty() || trajectory.getWayPointCount() < 2)
    return false;

  const auto& joint_group_name = trajectory.getGroupName();
  const auto* model_group = trajectory.getRobotModel()->getJointModelGroup(joint_group_name);
  if (!model_group)
  {
    RCLCPP_ERROR(logger_, "Failed to retrieve model group for joint group '%s'", joint_group_name.c_str());
    return false;
  }
  const auto& joint_names = model_group->getVariableNames();

  const std::size_t n = trajectory.getWayPointCount();
  RCLCPP_DEBUG(logger_, "Found '%zu' waypoints in trajectory in '%s'", n, getDescription().c_str());

  double max_velocity_scaling_factor = req.max_velocity_scaling_factor;
  double max_acceleration_scaling_factor = req.max_acceleration_scaling_factor;

  for (std::size_t i = 0; i < n; ++i)
  {
    moveit::core::RobotStatePtr state_ptr = trajectory.getWayPointPtr(i);
    double progress = (n > 1) ? static_cast<double>(i) / (n - 1) : 1.0;

    double velocity_scale = (progress > 0.8) ? 1.0 - 0.99 * (progress - 0.8) / 0.2 : 1.0;
    double acceleration_scale = velocity_scale;
    velocity_scale *= max_velocity_scaling_factor;
    acceleration_scale *= max_acceleration_scaling_factor;

    for (const std::string& joint_name : joint_names)
    {
      const auto* joint_model = trajectory.getRobotModel()->getJointModel(joint_name);
      if (!joint_model)
        continue;
      const auto& bounds = joint_model->getVariableBounds(joint_name);
      double max_vel = bounds.max_velocity_;
      double max_acc = bounds.max_acceleration_;
      double scaled_vel = max_vel * velocity_scale;
      double scaled_acc = max_acc * acceleration_scale;

      state_ptr->setVariableVelocity(joint_name, scaled_vel);
      state_ptr->setVariableAcceleration(joint_name, scaled_acc);
    }
  }

  return true;
}

}  // namespace mg400_ros2_bringup

PLUGINLIB_EXPORT_CLASS(mg400_ros2_bringup::FastStartSlowEndAdapter, planning_interface::PlanningResponseAdapter)