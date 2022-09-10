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

    # If we are in simulation mode we can use the state broadcaster UI which allows users to
    # change the joints manually.
    if is_simulation == 'true' or use_fake_hardware == 'true':
        joint_state_broadcaster_node = Node(
            package="joint_state_broadcaster_gui",
            executable="joint_state_broadcaster_gui",
        )
    else:

        joint_state_broadcaster_node = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )

    postion_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_trajectory_controller", "-c", "/controller_manager"],
    )

    # Delay creating the position trajectory controller until the joint_state_broadcast node has been started so that
    # the position trajectory controller can get the different TF frames from the broadcaster
    delay_position_trajectory_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[postion_trajectory_controller_spawner],
        )
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/controller_manager"],
    )

    # Delay creating the velocity controller until the joint_state_broadcast node has been started so that
    # the velocity controller can get the different TF frames from the broadcaster
    delay_velocity_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[velocity_controller_spawner],
        )
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(control_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_broadcaster_node)
    ld.add_action(delay_position_trajectory_controller_spawner_after_joint_state_broadcaster_spawner)
    ld.add_action(delay_velocity_controller_spawner_after_joint_state_broadcaster_spawner)
    return ld
