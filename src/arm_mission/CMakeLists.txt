cmake_minimum_required(VERSION 3.0.2)
project(arm_mission)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs arm_gui arm_grasp arm_perception roslaunch)
catkin_python_setup()
catkin_package()

roslaunch_add_file_check(launch)
install(PROGRAMS scripts/mission_control_node.py scripts/arm_mission_rqt DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_mission_logic.py)
  catkin_add_nosetests(test/test_mission_widget.py)
  add_rostest(test/mission_node.test)
  catkin_install_python(PROGRAMS test/test_mission_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
