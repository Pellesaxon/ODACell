#include <rclcpp/rclcpp.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

#include <chrono>

namespace mg400_analytical_ik
{
// URDF-derived constants for the analytical IK (will blow up if we change the model == very epic)
const double J1_OFFSET_X = -0.005;
const double J1_OFFSET_Z = 0.109;
const KDL::Vector V_L1_TO_J2(0.0435007595, -0.0357748756, 0.1189956826);
const KDL::Vector V_J2_TO_J3(-0.0010512570, 0.0357748756, 0.1750011643);
const KDL::Vector V_J3_TO_WRIST(0.1749697840, -0.0170000000, 0.0032518713);
const KDL::Vector V_WRIST_TO_TIP(0.0659996239, 0.0170000000, 0.0310008007);
const double L_UPPER_ARM = V_J2_TO_J3.Norm();
const double L_FOREARM = V_J3_TO_WRIST.Norm();

struct JointLimit
{
  double lower;
  double upper;
};

class MG400KinematicsPlugin : public kinematics::KinematicsBase
{
public:
  MG400KinematicsPlugin() = default;

  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                  const std::string& group_name, const std::string& base_frame,
                  const std::vector<std::string>& tip_frames, double search_discretization) override
  {
    storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);
    if (!jmg)
    {
      RCLCPP_FATAL(node->get_logger(), "Failed to get Joint Model Group '%s'", group_name.c_str());
      return false;
    }

    joint_names_ = jmg->getVariableNames();
    for (const auto& joint_name : joint_names_)
    {
      const moveit::core::VariableBounds& bounds = robot_model_->getVariableBounds(joint_name);
      joint_limits_.push_back({ bounds.min_position_, bounds.max_position_ });
    }

    if (!kdl_parser::treeFromUrdfModel(*robot_model.getURDF(), kdl_tree_))
    {
      RCLCPP_FATAL(node->get_logger(), "Could not initialize KDL tree");
      return false;
    }
    if (!kdl_tree_.getChain(base_frame_, getTipFrame(), kdl_chain_))
    {
      RCLCPP_FATAL(node->get_logger(), "Could not get KDL chain from tree");
      return false;
    }
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

    RCLCPP_INFO(node->get_logger(), "MG400 Analytical IK Plugin initialized successfully.");
    return true;
  }

  // --- INVERSE KINEMATICS (IK) ---
  // I can do this men inte linalg1
  bool calculateIK(const geometry_msgs::msg::Pose& ik_pose, std::vector<double>& solution,
                   moveit_msgs::msg::MoveItErrorCodes& error_code) const
  {

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    KDL::Frame pose_kdl;
    tf2::fromMsg(ik_pose, pose_kdl);

    if (std::abs(pose_kdl.M.UnitZ().z()) < 0.999)
    {
      RCLCPP_ERROR_ONCE(rclcpp::get_logger("mg400_ik"), "Target pose is unreachable. Tool Z-axis must be vertical.");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }

    KDL::Vector wrist_center_pos = pose_kdl.p - pose_kdl.M * V_WRIST_TO_TIP;
    double wrist_x_rel_j1 = wrist_center_pos.x() - J1_OFFSET_X;
    double wrist_y_rel_j1 = wrist_center_pos.y();
    double j1 = std::atan2(wrist_y_rel_j1, wrist_x_rel_j1);

    double r = std::hypot(wrist_x_rel_j1, wrist_y_rel_j1);
    double dx = r - V_L1_TO_J2.x();
    double dz = wrist_center_pos.z() - J1_OFFSET_Z - V_L1_TO_J2.z();
    double D_sq = dx * dx + dz * dz;

    if (D_sq > std::pow(L_UPPER_ARM + L_FOREARM, 2) + 1e-6 || D_sq < std::pow(L_UPPER_ARM - L_FOREARM, 2) - 1e-6)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("mg400_ik"), "Target is out of reach.");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }

    double cos_j3_arg = (D_sq - L_UPPER_ARM * L_UPPER_ARM - L_FOREARM * L_FOREARM) / (2.0 * L_UPPER_ARM * L_FOREARM);
    cos_j3_arg = std::max(-1.0, std::min(1.0, cos_j3_arg));

    double j3 = std::acos(cos_j3_arg); // Elbow up solution

    double s3 = std::sin(j3);
    double c3 = std::cos(j3);
    double j2 = std::atan2(dz, dx) - std::atan2(L_FOREARM * s3, L_UPPER_ARM + L_FOREARM * c3);

    double phi = std::atan2(pose_kdl.M(1, 0), pose_kdl.M(0, 0));
    double j4 = phi - j1;
    j4 = std::fmod(j4 + M_PI, 2.0 * M_PI) - M_PI;

    std::vector<double> current_solution = { j1, j2, j3, j4 };
    if (isSolutionValid(current_solution))
    {
      solution = current_solution;
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

      std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
      // Just for curiosity, remove if we want to keep the log clean
      /*
      RCLCPP_INFO(rclcpp::get_logger("mg400_ik"),
      "IK solution found in %ld ms: j1=%.3f, j2=%.3f, j3=%.3f, j4=%.3f",
      duration.count(), j1, j2, j3, j4);
      */

      return true;
    }

    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  // --- FORWARD KINEMATICS (FK) ---
  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override
  {
    if (joint_angles.size() != kdl_chain_.getNrOfJoints())
    {
      RCLCPP_ERROR(rclcpp::get_logger("mg400_fk"), "Joint angles vector size does not match KDL chain size.");
      return false;
    }

    KDL::JntArray jnt_pos_in(kdl_chain_.getNrOfJoints());
    for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
      jnt_pos_in(i) = joint_angles[i];

    poses.clear();
    for (const std::string& link_name : link_names)
    {
      if (link_name != getTipFrame())
      {
        RCLCPP_WARN(rclcpp::get_logger("mg400_fk"), "Only FK for the tip frame is supported.");
        continue;
      }

      KDL::Frame kdl_pose;
      if (fk_solver_->JntToCart(jnt_pos_in, kdl_pose) < 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("mg400_fk"), "KDL FK solver failed.");
        return false;
      }
      // CORRECTED: tf2::toMsg returns the message, it does not take an output parameter
      poses.push_back(tf2::toMsg(kdl_pose));
    }
    return true;
  }

  // Forward deprecated function to replacement.
  bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                     std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override
  {
    return searchPositionIK(ik_pose, ik_seed_state, 0.0, solution, error_code, options);
  }

  // Shut the compiler up by implementing ALL unused virtual functions
  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override
  {
    // Unused parameters for this analytic. solver
    [[maybe_unused]] const auto& seed = ik_seed_state;
    [[maybe_unused]] double t = timeout;
    [[maybe_unused]] const auto& opt = options;
    return calculateIK(ik_pose, solution, error_code);
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override
  {
    // Unused parameters for this analytic. solver
    [[maybe_unused]] const auto& seed = ik_seed_state;
    [[maybe_unused]] double t = timeout;
    [[maybe_unused]] const auto& limits = consistency_limits;
    [[maybe_unused]] const auto& opt = options;
    return calculateIK(ik_pose, solution, error_code);
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, std::vector<double>& solution, const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override
  {
    // Unused parameters for analytic. solver
    [[maybe_unused]] const auto& seed = ik_seed_state;
    [[maybe_unused]] double t = timeout;
    [[maybe_unused]] const auto& opt = options;
    if (calculateIK(ik_pose, solution, error_code))
    {
      solution_callback(ik_pose, solution, error_code);
      return true;
    }
    return false;
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution,
                        const IKCallbackFn& solution_callback, moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override
  {
    // Unused parameters analytic. solver
    [[maybe_unused]] const auto& seed = ik_seed_state;
    [[maybe_unused]] double t = timeout;
    [[maybe_unused]] const auto& limits = consistency_limits;
    [[maybe_unused]] const auto& opt = options;
    if (calculateIK(ik_pose, solution, error_code))
    {
      solution_callback(ik_pose, solution, error_code);
      return true;
    }
    return false;
  }

  const std::vector<std::string>& getJointNames() const override { return joint_names_; }
  const std::vector<std::string>& getLinkNames() const override { return getTipFrames(); }

private:
  bool isSolutionValid(const std::vector<double>& sol) const
  {
    if (sol.size() != joint_names_.size()) return false;
    for (size_t i = 0; i < sol.size(); ++i)
    {

      if (sol[i] < joint_limits_[i].lower - 1e-5 || sol[i] > joint_limits_[i].upper + 1e-5)
      {
        return false;
      }
    }
    return true;
  }

  std::vector<std::string> joint_names_;
  std::vector<JointLimit> joint_limits_;
  KDL::Chain kdl_chain_;
  KDL::Tree kdl_tree_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
};

}  // namespace mg400_analytical_ik

PLUGINLIB_EXPORT_CLASS(mg400_analytical_ik::MG400KinematicsPlugin, kinematics::KinematicsBase);