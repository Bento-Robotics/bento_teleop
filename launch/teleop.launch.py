import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    robot_namespace = LaunchConfiguration('robot_namespace')

    bento_teleop = Node(
        package='bento_teleop',
        executable='teleop_node',
        name='teleop_node',
        parameters=[
            PathJoinSubstitution([ FindPackageShare('bento_teleop'), 'parameter', 'teleop.yaml' ]),
        ],
        output='screen',
        emulate_tty=True,
        namespace=robot_namespace,
    )

    joystick = Node(
        package='joy_linux',
        executable='joy_linux_node',
        parameters=[
            {'autorepeat_rate': 20.0},
            {'coalesce_interval_ms': 50}
        ],
        output='screen',
        emulate_tty=True,
        namespace=[robot_namespace, "_opr"],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='bento',
            description='set namespace for robot nodes'
        ),
        bento_teleop,
        joystick,
    ])
