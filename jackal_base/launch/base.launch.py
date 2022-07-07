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
import xacro.xmlutils 


def generate_launch_description():
    # The base Jackal configuration has no accessories at all,
    # so nothing need be specified if config==base; the defaults as given
    # in the URDF suffice to define this config.

    bb2 = SetEnvironmentVariable('JACKAL_BB2', '1',
                                 condition=LaunchConfigurationEquals('config', 'front_bumblebee2'))
    flea3 = SetEnvironmentVariable('JACKAL_FLEA3', '1',
                                   condition=LaunchConfigurationEquals('config', 'front_flea3'))

    # The front_laser configuration of Jackal is sufficient for
    # basic gmapping and navigation. It is mostly the default
    # config, but with a SICK LMS100 series LIDAR on the front,
    # pointing forward.
    laser = SetEnvironmentVariable('JACKAL_LASER_3D', '1')
    laser_model = SetEnvironmentVariable('JACKAL_LASER_3D_MODEL', 'hdl32e')
    laser2 = SetEnvironmentVariable('JACKAL_LASER_SECONDARY', '1')

    config_arg = DeclareLaunchArgument('config', default_value='/home/rkreinin/jackal_ws/src/config_1.yaml')

    config_file = LaunchConfiguration('config')

    accessories_env = SetEnvironmentVariable('JACKAL_URDF_ACCESSORIES', '/home/rkreinin/jackal_ws/src/accessories.urdf.xacro')

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

    builder = Node(
        package='cpr_accessories_builder',
        executable='builder',
        arguments=[config_file, '/home/rkreinin/jackal_ws/src/accessories.urdf.xacro'],
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
                     rviz]))

    

    ld = LaunchDescription()
    #ld.add_action(bb2)
    #ld.add_action(flea3)
    #ld.add_action(laser)
    #ld.add_action(laser_model)
    #ld.add_action(laser2)
    #ld.add_action(micro_ros_agent)
    ld.add_action(config_arg)
    ld.add_action(accessories_env)
    ld.add_action(build_event)
    ld.add_action(builder)
    # ld.add_action(node_robot_state_publisher)
    # ld.add_action(node_controller_manager)
    # ld.add_action(spawn_controller)
    # ld.add_action(spawn_jackal_velocity_controller)
    # ld.add_action(launch_jackal_control)
    # ld.add_action(launch_jackal_teleop_base)
    # ld.add_action(launch_jackal_teleop_joy)
    # ld.add_action(rviz)
    return ld
