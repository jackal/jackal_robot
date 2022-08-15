import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    # Launch Arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution(
                [FindPackageShare('cpr_accessories_bringup'), 'config', 'empty.yaml']
        )
    )

    accessories_folder_arg = DeclareLaunchArgument(
        'accessories_folder',
        default_value=PathJoinSubstitution([os.path.expanduser('~'), 'jackal_accessories'])
    )

    # Launch Configurations
    config_file = LaunchConfiguration('config_file')
    accessories_folder = LaunchConfiguration('accessories_folder')

    accessories_launch_path = PathJoinSubstitution([accessories_folder, 'accessories.launch.py'])
    # Get URDF via xacro
    robot_description_command = [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'), 'urdf', 'jackal.urdf.xacro']
            ),
            ' ',
            'accessories:=',
            PathJoinSubstitution([accessories_folder, 'accessories.urdf.xacro']),
        ]

    # Accessories builder
    builder = Node(
        package='cpr_accessories_builder',
        executable='builder',
        arguments=[config_file, accessories_folder],
        output='screen',
    )

    jackal_bringup_group_action = GroupAction([
        # Jackal Description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('jackal_description'),
                     'launch',
                     'description.launch.py']
                )
            ),
            launch_arguments=[('robot_description_command', robot_description_command)]
        ),

        # Jackal Control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('jackal_control'), 'launch', 'control.launch.py']
            )),
            launch_arguments=[('robot_description_command', robot_description_command)]
        ),

        # Base Teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('jackal_control'), 'launch', 'teleop_base.launch.py']
            ))
        ),

        # Joy Teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('jackal_control'), 'launch', 'teleop_joy.launch.py']
            ))
        ),

        # Wireless Watcher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('wireless_watcher'), 'launch', 'watcher.launch.py']
            )),
            launch_arguments=[('connected_topic', 'wifi_connected')]
        ),

        # Accessories
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(accessories_launch_path)
        ),

        # MicroROS Agent
        # Node(
        #     package='micro_ros_agent',
        #     executable='micro_ros_agent',
        #     arguments=['serial', '--dev', '/dev/jackal'],
        #     output='screen',
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])

    build_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=builder,
            on_exit=[jackal_bringup_group_action]
        )
    )

    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(accessories_folder_arg)
    ld.add_action(builder)
    ld.add_action(build_event)
    return ld
