cmake_minimum_required(VERSION 3.0.2)
project(arm_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  roscpp
  rospy
  roslaunch
  rosbag
  urdf
  dynamic_reconfigure
  diagnostic_msgs
)

roslaunch_add_file_check(launch)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/ArmInterface.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arm_interface
#  CATKIN_DEPENDS ament_cmake dynamixel_sdk roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/arm_interface.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(bulk_read src/bulk_read.cpp)
add_dependencies(bulk_read ${catkin_EXPORTED_TARGETS} dynamixel_sdk)
target_link_libraries(bulk_read ${catkin_LIBRARIES} dynamixel_sdk)

add_executable(motor_diag src/motor_diag.cpp)
add_dependencies(motor_diag ${catkin_EXPORTED_TARGETS} dynamixel_sdk)
target_link_libraries(motor_diag ${catkin_LIBRARIES} dynamixel_sdk)
target_include_directories(motor_diag PRIVATE include)

add_executable(find_limits src/find_limits.cpp)
add_dependencies(find_limits ${catkin_EXPORTED_TARGETS} dynamixel_sdk)
target_link_libraries(find_limits ${catkin_LIBRARIES} dynamixel_sdk)

add_executable(robot_interface src/robot_interface.cpp)
add_dependencies(robot_interface ${catkin_EXPORTED_TARGETS} dynamixel_sdk ${PROJECT_NAME}_gencfg)
target_link_libraries(robot_interface ${catkin_LIBRARIES} dynamixel_sdk)

# Second, velocity-commanded hardware interface (M5) -- see
# docs/sim_to_real.md. Separate executable rather than a mode flag on
# robot_interface: different Dynamixel operating mode, different
# command-domain semantics (a stale velocity command keeps driving the
# motor, unlike a stale position command), different safety-critical
# timeout behavior.
add_executable(robot_interface_velocity src/robot_interface_velocity.cpp)
add_dependencies(robot_interface_velocity ${catkin_EXPORTED_TARGETS} dynamixel_sdk ${PROJECT_NAME}_gencfg)
target_link_libraries(robot_interface_velocity ${catkin_LIBRARIES} dynamixel_sdk)

# ─── Simulation Validator Node (for control validation in sim) ────────────────
add_executable(arm_sim_validator_node src/arm_sim_validator_node.cpp)
add_dependencies(arm_sim_validator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_sim_validator_node ${catkin_LIBRARIES})

add_executable(replay_bag src/run_bagfile.cpp)
add_dependencies(replay_bag ${catkin_EXPORTED_TARGETS})
target_link_libraries(replay_bag ${catkin_LIBRARIES})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/arm_interface_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
## Commented out: scripts/arm_gazebo_bridge_node.py does not exist in this
## package, which fails catkin's CMake configure step entirely (M0 finding).
# catkin_install_python(PROGRAMS
#   scripts/arm_gazebo_bridge_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test-watchdog test/test_watchdog.cpp)
catkin_add_gtest(${PROJECT_NAME}-test-gripper-conversion test/test_gripper_conversion.cpp)
catkin_add_gtest(${PROJECT_NAME}-test-estop test/test_estop.cpp)
catkin_add_gtest(${PROJECT_NAME}-test-drive-state test/test_drive_state.cpp)
catkin_add_gtest(${PROJECT_NAME}-test-motor-status test/test_motor_status.cpp)
catkin_add_gtest(${PROJECT_NAME}-test-motor-health test/test_motor_health.cpp)
catkin_add_gtest(${PROJECT_NAME}-test-position-gains test/test_position_gains.cpp)

## Python tests (URDF vs. hardware encoder limits)
catkin_add_nosetests(test/test_limits_consistent.py)
catkin_add_nosetests(test/test_estop_cli.py)
