cmake_minimum_required(VERSION 2.8.3)
project(jackal_bringup)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch/mcu.launch)
roslaunch_add_file_check(launch/teleop.launch)
