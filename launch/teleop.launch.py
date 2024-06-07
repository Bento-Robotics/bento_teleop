import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    bento_teleop = Node(
        package='bento_teleop',
        executable='teleop_node',
        name='teleop_node',
        parameters=[ PathJoinSubstitution([ FindPackageShare('bento_teleop'), 'parameter', 'teleop.yaml' ]) ],
        namespace=EnvironmentVariable( 'EDU_ROBOT_NAMESPACE', default_value="bento" ),
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        bento_teleop
    ])
