cmake_minimum_required(VERSION 3.0.2)
project(arm_grasp)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs message_generation roslaunch)
catkin_python_setup()
add_service_files(FILES Grasp.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

roslaunch_add_file_check(launch)
install(PROGRAMS scripts/grasp_sequencer_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_grasp_plan.py)
  catkin_add_nosetests(test/test_grasp_tree.py)
endif()
