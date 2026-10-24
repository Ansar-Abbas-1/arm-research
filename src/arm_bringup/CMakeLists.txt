cmake_minimum_required(VERSION 3.0.2)
project(arm_bringup)

find_package(catkin REQUIRED COMPONENTS roslaunch)
catkin_package()

# Every launch file must parse and resolve its includes and args (roslaunch's own check).
roslaunch_add_file_check(launch)
install(DIRECTORY launch config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_bringup.py)
endif()
