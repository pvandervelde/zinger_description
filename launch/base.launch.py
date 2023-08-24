from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([get_package_share_directory('zinger_description'), "urdf", 'base.xacro']),
            " ",
            "is_simulation:=",
            is_simulation,
             " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Takes the joint positions from the 'joint_state' topic and updates the position of the robot with tf2.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            robot_description,
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    return ld
