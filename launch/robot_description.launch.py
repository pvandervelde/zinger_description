from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time')
]


def generate_launch_description():
    pkg_robot_description = get_package_share_directory('cratebot_description')
    xacro_file = PathJoinSubstitution([pkg_robot_description,
                                       'urdf',
                                       'base.xacro'])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': ParameterValue(Command(['xacro', ' ', xacro_file, ' ', 'gazebo:=ignition']), value_type=str)},
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    return ld