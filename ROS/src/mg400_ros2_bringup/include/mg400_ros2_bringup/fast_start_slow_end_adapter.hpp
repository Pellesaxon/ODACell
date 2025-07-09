#pragma once

#include <moveit/planning_interface/planning_response_adapter.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <rclcpp/logger.hpp>

namespace mg400_ros2_bringup
{
class FastStartSlowEndAdapter : public planning_interface::PlanningResponseAdapter
{
public:
  FastStartSlowEndAdapter();

  ~FastStartSlowEndAdapter() override = default;

  [[nodiscard]] std::string getDescription() const override;

  void adapt(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest& req,
             planning_interface::MotionPlanResponse& res) const override;

private:
  bool applyFastStartSlowEnd(const planning_interface::MotionPlanRequest& req,
                             robot_trajectory::RobotTrajectory& trajectory) const;
  rclcpp::Logger logger_;
};

}  // namespace mg400_ros2_bringup