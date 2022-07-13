from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'), 'urdf', 'jackal.urdf.xacro']
            ),
            ' ',
            'name:=jackal',
            ' ',
            'prefix:=''',
            ' ',
        ]
    ), value_type=str)

    robot_description = {'robot_description': robot_description_content}

    config_jackal_velocity_controller = PathJoinSubstitution(
        [FindPackageShare('jackal_control'),
        'config',
        'control.yaml'],
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, config_jackal_velocity_controller],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    spawn_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    spawn_jackal_velocity_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['jackal_velocity_controller'],
        output='screen',
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/jackal'],
        output='screen',
    )

    # Launch jackal_control/control.launch.py which is just robot_localization.
    launch_jackal_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'launch', 'control.launch.py'])))

    # Launch jackal_control/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_jackal_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'launch', 'teleop_base.launch.py'])))

    # Launch jackal_control/teleop_joy.launch.py which is tele-operation using a physical joystick.
    launch_jackal_teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'launch', 'teleop_joy.launch.py'])))

    ld = LaunchDescription()
    ld.add_action(micro_ros_agent)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_controller_manager)
    ld.add_action(spawn_controller)
    ld.add_action(spawn_jackal_velocity_controller)
    ld.add_action(launch_jackal_control)
    ld.add_action(launch_jackal_teleop_base)
    ld.add_action(launch_jackal_teleop_joy)

    return ld
