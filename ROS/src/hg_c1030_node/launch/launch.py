# hg_c1030_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the node for the HG-C1030-P laser sensor.
    This launch file is configurable via launch arguments.
    """
    
    default_port = '/dev/ttyACM0'
    default_num_sensors = '2'
    default_frame_ids = ["distance_sensor_0", "distance_sensor_1"]

    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'port',
            default_value=default_port,
            description='The serial port the Arduino is connected to.'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'frame_ids',
            default_value=[str(item) for item in default_frame_ids], # Ensure default is list of strings
            description='A list of TF frame_ids for the sensor readings.'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'num_sensors',
            default_value=default_num_sensors,
            description='The number of sensors to use.'
        )
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
            # Pass the list parameter as a direct Python list.
            # This is the most reliable way to handle list parameters.
            'frame_ids': default_frame_ids 
        }]
    )

    return LaunchDescription(declared_arguments + [sensor_node])