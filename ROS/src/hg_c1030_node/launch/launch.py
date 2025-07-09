from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the node for the HG-C1030-P laser sensor.
    This launch file is configurable via launch arguments.
    """

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='The serial port the Arduino is connected to.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser_distance_sensor',
            description='The TF frame_id for the sensor readings. Should be a link on the robot.'
        )
    )

    port = LaunchConfiguration('port')
    frame_id = LaunchConfiguration('frame_id')

    sensor_node = Node(
        package='hg_c1030_node',
        executable='hg_sensor_node',
        name='hg_sensor_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': port,
            'frame_id': frame_id
            # Possibly implement dynamic baud rate later
        }]
    )

    return LaunchDescription(declared_arguments + [sensor_node])