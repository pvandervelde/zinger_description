from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
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

    ld = LaunchDescription(ARGUMENTS)

    # When running in Ignition / Gazebo it runs a different controller manager so we don't need this one?
    if is_simulation != 'true':
        add_controller_manager(
            is_simulation=is_simulation,
            use_fake_hardware=use_fake_hardware,
            fake_sensor_commands=fake_sensor_commands,
            ld=ld)

    joint_state_broadcaster_node = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    ld.add_action(joint_state_broadcaster_node)


    postion_trajectory_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'position_trajectory_controller'],
        output='screen'
    )

    # Delay creating the position trajectory controller until the joint_state_broadcast node has been started so that
    # the position trajectory controller can get the different TF frames from the broadcaster
    delay_position_trajectory_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[postion_trajectory_controller_spawner],
        )
    )
    ld.add_action(delay_position_trajectory_controller_spawner_after_joint_state_broadcaster_spawner)


    velocity_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'velocity_controller'],
        output='screen'
    )

    # Delay creating the velocity controller until the joint_state_broadcast node has been started so that
    # the velocity controller can get the different TF frames from the broadcaster
    delay_velocity_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[velocity_controller_spawner],
        )
    )
    ld.add_action(delay_velocity_controller_spawner_after_joint_state_broadcaster_spawner)

    return ld

def add_controller_manager(is_simulation: str, use_fake_hardware: str, fake_sensor_commands: str, ld: LaunchDescription):
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([get_package_share_directory('cratebot_description'), "urdf", 'base.xacro']),
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

    ld.add_action(control_node)
