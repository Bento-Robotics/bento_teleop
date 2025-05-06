import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    bento_teleop = Node(
        package='bento_teleop',
        executable='arm_teleop_node',
        name='arm_teleop_node',
        parameters=[ PathJoinSubstitution([ FindPackageShare('bento_teleop'), 'parameter', 'arm_teleop.yaml' ]) ],
        namespace=EnvironmentVariable( 'OPERATOR_NAMESPACE', default_value="box_opr" ),
        output='screen',
        emulate_tty=True,
    )

    joystick = Node(
        package='joy_linux',
        executable='joy_linux_node',
        parameters=[
            {'autorepeat_rate': 20.0},
            {'coalesce_interval_ms': 50}
        ],
        namespace=EnvironmentVariable( 'OPERATOR_NAMESPACE', default_value="box_opr" ),
    )

    return LaunchDescription([
        bento_teleop,
        joystick,
    ])
