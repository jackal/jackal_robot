from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    config_file_arg = DeclareLaunchArgument('config_file',
                                       default_value=PathJoinSubstitution(
                                           [FindPackageShare('cpr_accessories_bringup'),
                                            'config', 'empty.yaml']))
    accessories_path_arg = DeclareLaunchArgument('accessories_path', default_value='/home/rkreinin/jackal_ws/src')

    config_file = LaunchConfiguration('config_file')
    accessories_path = LaunchConfiguration('accessories_path')

    accessories_launch_path = PathJoinSubstitution([accessories_path, 'accessories.launch.py'])
    # Get URDF via xacro
    robot_description_content = ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'), 'urdf', 'jackal.urdf.xacro']
            ),
            ' ',
            'accessories:=',
            PathJoinSubstitution([accessories_path, 'accessories.urdf.xacro']),
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

    launch_jackal_accessories = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(accessories_launch_path)
    )

    builder = Node(
        package='cpr_accessories_builder',
        executable='builder',
        arguments=[config_file, PathJoinSubstitution([accessories_path, 'accessories'])],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    build_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=builder,
            on_exit=[node_robot_state_publisher,
                     node_controller_manager,
                     spawn_controller,
                     spawn_jackal_velocity_controller,
                     launch_jackal_control,
                     launch_jackal_teleop_base,
                     launch_jackal_teleop_joy,
                     launch_jackal_accessories,
                     rviz]))

    

    ld = LaunchDescription()
    #ld.add_action(micro_ros_agent)
    ld.add_action(config_file_arg)
    ld.add_action(accessories_path_arg)
    ld.add_action(build_event)
    ld.add_action(builder)
    return ld
