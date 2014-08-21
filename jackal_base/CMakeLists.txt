cmake_minimum_required(VERSION 2.8.3)
project(jackal_base)

find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface roscpp roslaunch rosserial_server teleop_twist_joy)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(jackal_node src/jackal_base)
target_link_libraries(jackal_node ${catkin_LIBRARIES})
add_dependencies(jackal_node jackal_msgs_gencpp rosserial_msgs_gencpp)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch/base.launch)
