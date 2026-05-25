"""
Launch file for the agrobot PLC bridge node.

Default config is loaded from config/plc_bridge.yaml inside the installed
package.  Override with the config_file launch argument:

    ros2 launch agrobot_plc_bridge plc_bridge.launch.py \
        config_file:=/path/to/my_config.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    default_config = PathJoinSubstitution([
        FindPackageShare('agrobot_plc_bridge'),
        'config',
        'plc_bridge.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Absolute path to the PLC bridge YAML parameter file',
        ),
        Node(
            package='agrobot_plc_bridge',
            executable='plc_bridge',
            name='plc_bridge',
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
            emulate_tty=True,
        ),
    ])
