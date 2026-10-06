cmake_minimum_required(VERSION 3.0.2)
project(arm_camera)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)
catkin_python_setup()
catkin_package()

roslaunch_add_file_check(launch)

install(PROGRAMS
  scripts/hand_eye_collector.py
  scripts/hand_eye_solve.py
  scripts/calibrate_intrinsics.py
  scripts/hand_eye_tf_publisher.py
  scripts/calibration_overlay_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
