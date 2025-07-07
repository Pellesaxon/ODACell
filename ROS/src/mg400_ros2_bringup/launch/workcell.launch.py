import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import pprint

def generate_launch_description():
    # Declare arguments for the launch file
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "driver_only", default_value="false",
            description="Whether to launch only the driver node without MoveIt or RViz."
        )
    )

    # =================================================================================
    # ===                      MOVEIT CONFIGURATION                                 ===
    # =================================================================================
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.start_state_max_bounds_error": 1.0,
        "trajectory_execution.allowed_start_tolerance": 1.0,
        # "execution_duration_monitoring": False,
        }
    
    moveit_config = (
        MoveItConfigsBuilder(robot_name="mg400", package_name="mg400_ros2_bringup")
        .robot_description(file_path="urdf/mg400.urdf.xacro")
        .robot_description_semantic(file_path="config/mg400.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )


    # =================================================================================
    # ===                     NODES TO LAUNCH                                       ===
    # =================================================================================

    # ---- MG400 Driver Node ----
    driver_node = Node(
        package="mg400_ros2_bringup",
        executable="mg400_driver_node",
        output="screen",
    )

    # ---- Robot State Publisher ----
    # Publishes TF transforms for the robot based on joint states.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
        remappings=[
            ("joint_states", "mg400/joint_states"),
        ]
    )

    # ---- RViz ----
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("mg400_ros2_bringup"), "config", "moveit.rviz"]
        )],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=UnlessCondition(LaunchConfiguration("driver_only")),
    )

    # ---- MoveGroup Node ----
    # The main MoveIt node for planning and execution.
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), trajectory_execution, {"use_sim_time": False}],
        condition=UnlessCondition(LaunchConfiguration("driver_only")),
        remappings=[("joint_states", "mg400/joint_states")],
    )


    # The final list of nodes to launch
    nodes_to_start = [
        driver_node,
        robot_state_publisher_node,
        rviz_node,
        move_group_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)