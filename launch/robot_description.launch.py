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
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time')
]


def generate_launch_description():
    is_simulation = LaunchConfiguration("use_sim_time")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([get_package_share_directory('cratebot_description'), "urdf", 'base.xacro']),
            " ",
            "is_simulation:=",
            is_simulation,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            get_package_share_directory('cratebot_description'),
            "config",
            'cratebot.yaml',
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controllers, "-c", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(control_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(delay_robot_controller_spawner_after_joint_state_broadcaster_spawner)
    return ld
