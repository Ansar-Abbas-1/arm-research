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
  scripts/aruco_detector_node.py
  scripts/aruco_distance_report.py
  scripts/make_aruco_markers.py
  scripts/make_charuco_board.py
  scripts/charuco_mount_calibration.py
  scripts/charuco_mount_solve.py
  scripts/hand_eye_tf_publisher.py
  scripts/calibration_overlay_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
