from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Get URDF via xacro
    robot_description_command = [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'), 'urdf', 'jackal.urdf.xacro']
            ),
            ' ',
        ]

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

        # Accessories
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('jackal_robot'), 'launch', 'accessories.launch.py']
            ))
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

        # Diagnostics
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('jackal_robot'), 'launch', 'diagnostics.launch.py']
            ))
        ),

        # Wireless Watcher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('wireless_watcher'), 'launch', 'watcher.launch.py']
            )),
            launch_arguments=[('connected_topic', 'wifi_connected')]
        ),

        # MicroROS Agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/jackal'],
            output='screen'),

        # Set ROS_DOMAIN_ID
        ExecuteProcess(
            cmd=[
                ['export ROS_DOMAIN_ID=0;'],
                [FindExecutable(name='ros2'),
                 ' service call /set_domain_id ',
                 ' jackal_msgs/srv/SetDomainId ',
                 '"domain_id: ',
                 EnvironmentVariable('ROS_DOMAIN_ID', default_value='0'),
                 '"']
            ],
            shell=True,
        ),
    ])

    ld = LaunchDescription()
    ld.add_action(jackal_bringup_group_action)
    return ld
