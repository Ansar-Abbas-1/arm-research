cmake_minimum_required(VERSION 3.0.2)
project(arm_perception)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)
catkin_python_setup()
catkin_package()

roslaunch_add_file_check(launch)

install(PROGRAMS
  scripts/marker_localizer_node.py
  scripts/marker_pose_cli.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_marker_logic.py)
  catkin_add_nosetests(test/test_marker_config.py)
endif()
