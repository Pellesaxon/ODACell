import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import launch_param_builder

def generate_launch_description():
    # Declare arguments for the launch file
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("use_moveit", default_value="false", description="Whether to launch MoveIt components.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("use_rviz", default_value="true", description="Whether to start RViz.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("use_mock", default_value="false", description="Whether to use mock hardware.")
    )


    def launch_setup(context, *args, **kwargs):
        # Initialize Arguments from launch context
        use_moveit = LaunchConfiguration("use_moveit")
        use_rviz = LaunchConfiguration("use_rviz")
        use_mock = LaunchConfiguration("use_mock")

        # -------------------------------------------------------------------
        # ---- STEP 1: Build the MoveIt configuration object and dictionary ---
        # -------------------------------------------------------------------
        
        # Use MoveItConfigsBuilder to process all config files and generate parameters
        moveit_config = (
            MoveItConfigsBuilder(
                robot_name="mg400",
                package_name="mg400_ros2_bringup"
            )
            # This processes the xacro and applies the prefix
            .robot_description(
                file_path="urdf/mg400.urdf.xacro", mappings={"use_mock": use_mock}
            )
            .robot_description_semantic(
                file_path="config/mg400.srdf",
            )
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(
                pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
            )
            .to_moveit_configs()
        )
        
        
        # Create a dictionary of MoveIt parameters for the move_group node
        moveit_params_dict = moveit_config.to_dict()
        moveit_params_dict.update({
            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
            "trajectory_execution.allowed_goal_duration_margin": 0.5,
            "trajectory_execution.allowed_start_tolerance": 0.01,
        })

        # -------------------------------------------------------------------
        # ---- STEP 2: Define the nodes for the entire system -------------
        # -------------------------------------------------------------------

        # The ros2_control controller manager configuration file
        ros2_controllers_path = PathJoinSubstitution(
            [FindPackageShare("mg400_ros2_bringup"), "config", "ros2_controllers.yaml"]
        )

        # The ros2_control manager node
        # It gets the prefixed URDF from the moveit_config object
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit_config.robot_description, ros2_controllers_path],
            output="screen",
        )

        # The MG400 driver node
        driver_node = Node(
            package="mg400_ros2_bringup",
            executable="mg400_driver_node",
            output="screen",
            condition=UnlessCondition(use_mock),
        )

        # Robot state publisher
        # It also gets the same prefixed URDF from the moveit_config object
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
        )

        # Controller Spawners
        spawner_args = ["--controller-manager", PathJoinSubstitution(['/', 'controller_manager'])]

        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster", 
            ] + spawner_args,
        )

        robot_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_trajectory_controller",
            ] + spawner_args,
        )
        # The move_group node
        # It gets its parameters from the dictionary we created earlier
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_params_dict],
        )

        # RViz
        rviz_config_file = PathJoinSubstitution(
            [FindPackageShare("mg400_ros2_bringup"), "config", "moveit.rviz"]
        )
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            # Pass all necessary parameters for the MoveIt MotionPlanning display
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
            condition=IfCondition(use_rviz),
        )

        # Create a list of all nodes to be returned
        nodes_to_start = [
            driver_node,
            control_node,
            robot_state_publisher_node,
            robot_controller_spawner,
            joint_state_broadcaster_spawner,
            move_group_node,
            rviz_node,
        ]

        return nodes_to_start

    # The final launch description that sets up arguments and calls the main function
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])