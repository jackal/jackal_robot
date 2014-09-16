cmake_minimum_required(VERSION 2.8.3)
project(jackal_base)

find_package(catkin REQUIRED COMPONENTS
  controller_manager hardware_interface jackal_msgs roscpp roslaunch
  roslint rosserial_server sensor_msgs teleop_twist_joy)
find_package(Boost)

catkin_package()

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(jackal_node src/jackal_base src/jackal_hardware)
target_link_libraries(jackal_node ${catkin_LIBRARIES})
add_dependencies(jackal_node jackal_msgs_gencpp rosserial_msgs_gencpp)

add_executable(${PROJECT_NAME}_simple_joy_node src/simple_joy_node.cpp)
target_link_libraries(${PROJECT_NAME}_simple_joy_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_simple_joy_node
  PROPERTIES OUTPUT_NAME simple_joy_node PREFIX "")
add_dependencies(${PROJECT_NAME}_simple_joy_node jackal_msgs_gencpp)

add_executable(${PROJECT_NAME}_sixpair src/sixpair.c)
target_link_libraries(${PROJECT_NAME}_sixpair usb ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_sixpair
  PROPERTIES OUTPUT_NAME sixpair PREFIX "")

install(TARGETS jackal_node
                ${PROJECT_NAME}_simple_joy_node
                ${PROJECT_NAME}_sixpair
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch/base.launch)
roslint_cpp()
roslint_add_test()
