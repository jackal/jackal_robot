#!/usr/bin/env python3

import os

import robot_upstart

import sys


def help():
    print('Jackal robot_upstart install script.')
    print('Usage: install.py <ROS_DOMAIN_ID>')
    print('ROS_DOMAIN_ID: optional, defaults to 0')


argc = len(sys.argv)

domain_id = 0
if argc == 2:
    try:
        domain_id = int(sys.argv[1])
    except ValueError:
        print('Invalid ROS_DOMAIN_ID {0}'.format(sys.argv[1]))
        help()
        sys.exit(1)

print('Installing Jackal. ROS_DOMAIN_ID={0}'.format(domain_id))

j = robot_upstart.Job(name='ros2',
                      rmw='rmw_fastrtps_cpp',
                      workspace_setup=os.environ['ROBOT_SETUP'],
                      ros_domain_id=domain_id)

j.symlink = True
j.add(package='jackal_robot', filename='launch/bringup.launch.py')
j.install()
