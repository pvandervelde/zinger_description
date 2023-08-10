from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='use_sim_time'
    ),
    DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Start robot with fake hardware mirroring command to its states.",
    ),
    DeclareLaunchArgument(
        "fake_sensor_commands",
        default_value="false",
        description="Enable fake command interfaces for sensors used for simple simulations. Used only if 'use_fake_hardware' parameter is true.",
    ),
]

def generate_launch_description():
    is_simulation = LaunchConfiguration("use_sim_time")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")

    pkg_robot_description = get_package_share_directory(
        'zinger_description')

    base_launch = PathJoinSubstitution(
        [pkg_robot_description, 'launch', 'base.launch.py'])
    controllers_launch = PathJoinSubstitution(
        [pkg_robot_description, 'launch', 'controllers.launch.py'])

    base_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([base_launch]),
        launch_arguments=[
            ('use_sim_time', is_simulation),
            ('use_fake_hardware', use_fake_hardware),
            ('fake_sensor_commands', fake_sensor_commands)]
    )

    controllers_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([controllers_launch]),
        launch_arguments=[
            ('use_sim_time', is_simulation),
            ('use_fake_hardware', use_fake_hardware),
            ('fake_sensor_commands', fake_sensor_commands)]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(base_launch_include)
    ld.add_action(controllers_launch_include)
    return ld
