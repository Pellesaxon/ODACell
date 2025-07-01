import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ----------
    # DECLARE LAUNCH ARGUMENTS
    # ----------
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="mg400",
            description="Name of the robot, used for multi-robot setups for namespacing and shared memory.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.6",
            description="IP address of the MG400 robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to start RViz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="mg400_",
            description="Prefix for the robot name, used for namespacing.",
        )
    )


    # ----------
    # INITIALIZE LAUNCH ARGUMENTS
    # ----------
    robot_name = LaunchConfiguration("robot_name")
    robot_ip = LaunchConfiguration("robot_ip")
    use_rviz = LaunchConfiguration("use_rviz")
    prefix = LaunchConfiguration("prefix")

    # ----------
    # GET FILE PATHS AND PARAMETERS
    # ----------
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mg400_ros2_bringup"), "urdf", "mg400.urdf.xacro"]
            ),
            " ",
            "robot_name:=",
            robot_name,
            " ",
            "prefix:=",
            "mg400"
        ]
    )

    # ** THE FIX IS HERE **
    # The correct type is 'str', not 'string'.
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Get the controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("mg400_ros2_bringup"),
            "config",
            "mg400_controllers.yaml",
        ]
    )
    
    # Get the RViz configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("mg400_ros2_bringup"), "config", "workcell.rviz"]
    )

    # ----------
    # DEFINE NODES
    # ----------
    # The MG400 driver node (handles TCP and shared memory)
    mg400_driver_node = Node(
        package="mg400_ros2_bringup",
        executable="mg400_driver_node",
        name="mg400_driver_node",
        namespace=robot_name,
        parameters=[{"robot_name": robot_name, "robot_ip": robot_ip}],
        output="screen",
    )

    # The ros2_control manager node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=robot_name,
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    # The robot_state_publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_name,
        output="screen",
        parameters=[robot_description],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )
    
    # ----------
    # DEFINE CONTROLLER SPAWNERS
    # ----------
    # Define common spawner arguments, including the namespace for the controller manager
    spawner_args = ["--controller-manager", PathJoinSubstitution(['/', robot_name, 'controller_manager'])]

    # Load the joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"] + spawner_args,
    )

    # Load the joint_trajectory_controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"] + spawner_args,
    )

    # ----------
    # DEFINE EVENT HANDLERS FOR SEQUENTIAL STARTUP
    # ----------
    # Delay the broadcaster until the control_node is up and running
    delay_jsb_after_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Delay the main controller until the broadcaster is ready
    delay_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    
    nodes_to_start = [
        mg400_driver_node,
        control_node,
        robot_state_pub_node,
        rviz_node,
        delay_jsb_after_control_node,
        delay_controller_after_jsb,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)