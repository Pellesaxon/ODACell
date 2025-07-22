import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import pprint

MOCK_NAME = "mg400_mock"
MOCK_IP = "127.0.0.1"

DEFAULT_ROBOT_NAME = "mg400"
DEFAULT_ROBOT_IP = "192.168.1.6"

DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_NUM_SENSORS = '2'
DEFAULT_FRAME_IDS = ["distance_sensor_0", "distance_sensor_1"]


def generate_launch_description():
    # Declare arguments for the launch file
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "driver_only", default_value="false",
            description="Whether to launch only the driver node without MoveIt or RViz."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("use_mock", default_value="false",
            description="Use a mock implementation of the MG400 driver instead of the real one."
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", default_value=DEFAULT_ROBOT_IP,
            description="IP address of the MG400 robot. Default is '192.168.1.6'"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name", default_value=DEFAULT_ROBOT_NAME,
            description="Name of the robot. Default is 'mg400'."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'port',
            default_value=DEFAULT_PORT,
            description='The serial port the Arduino is connected to.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'frame_ids',
            default_value=[str(item) for item in DEFAULT_FRAME_IDS], # Ensure default is list of strings
            description='A list of TF frame_ids for the sensor readings.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'num_sensors',
            default_value=DEFAULT_NUM_SENSORS,
            description='The number of sensors to use.'
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
            # pipelines=["stomp", "chomp", "pilz_industrial_motion_planner", "ompl"]
            pipelines=["chomp"]
        )
        .to_moveit_configs()
    )


    # =================================================================================
    # ===                     NODES TO LAUNCH                                       ===
    # =================================================================================

    # Launch mock server if use_mock is true
    mock_server_process = ExecuteProcess(
    cmd=[
        "python3",
        #"-u",
        PathJoinSubstitution([
            FindPackageShare("mg400_ros2_bringup"),
            "mock",
            "mock.py"
        ])
    ],
    output="screen",
    condition=IfCondition(LaunchConfiguration("use_mock")),
    emulate_tty=True
    )
    


    # ---- MG400 Driver Node ----
    driver_node_real = Node(
        package="mg400_ros2_bringup",
        executable="mg400_driver_node",
        output="screen",
        parameters=[{
        "robot_ip": LaunchConfiguration("robot_ip"),
        "robot_name": LaunchConfiguration("robot_name")}],
        condition=UnlessCondition(LaunchConfiguration("use_mock")),
    )

    # Mock node
    driver_node_mock = Node(
        package="mg400_ros2_bringup",
        executable="mg400_driver_node",
        output="screen",
        parameters=[{
            "robot_ip": MOCK_IP,
            "robot_name": MOCK_NAME,
            "use_sim_time": False
        }],
        condition=IfCondition(LaunchConfiguration("use_mock")),
    )


    # ---- Robot State Publisher ----
    # Publishes TF transforms for the robot based on joint states.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ---- RViz ----
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("mg400_ros2_bringup"), "config", "moveit.rviz"],),
            "--ros-args", "--log-level", "fatal"],
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
    )
    
    port = LaunchConfiguration('port')
    num_sensors = LaunchConfiguration('num_sensors')
    
    sensor_node = Node(
        package='hg_c1030_node',
        executable='hg_sensor_node',
        name='hg_sensor_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': port,
            'num_sensors': num_sensors,
            'frame_ids': DEFAULT_FRAME_IDS
        }]
    )
    
    precision_homing_node = Node(
        package='mg400_ros2_bringup',
        executable='precision_homing_node',
        output='screen',
    )

    world_creator = Node(
        package='mg400_ros2_bringup',
        executable='world_creator',
        output='screen',
    )

    benchmark_node = Node(
        package='mg400_ros2_bringup',
        executable='benchmark',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration("driver_only")),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": False}
        ]
    )

    # The final list of nodes to launch
    nodes_to_start = [
        mock_server_process,
        driver_node_real,
        driver_node_mock,
        robot_state_publisher_node,
        rviz_node,
        move_group_node,
        sensor_node,
        precision_homing_node,
        world_creator,
        benchmark_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)