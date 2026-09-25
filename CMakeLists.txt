cmake_minimum_required(VERSION 3.0.2)
project(arm_trajectory)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  arm_kinematics
  urdf
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  actionlib
  actionlib_msgs
  roslaunch
  message_generation
)
find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  GenerateTrajectory.srv
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arm_trajectory
  CATKIN_DEPENDS roscpp arm_kinematics urdf geometry_msgs sensor_msgs trajectory_msgs control_msgs actionlib actionlib_msgs message_runtime
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(arm_trajectory
  src/quintic_time_scaling.cpp
  src/joint_space_trajectory.cpp
  src/cartesian_trajectory.cpp
)
add_dependencies(arm_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_trajectory
  ${catkin_LIBRARIES}
)

add_executable(trajectory_service_node src/trajectory_service_node.cpp)
add_dependencies(trajectory_service_node ${PROJECT_NAME}_gencpp)
target_link_libraries(trajectory_service_node
  arm_trajectory
  ${catkin_LIBRARIES}
)

add_executable(trajectory_executor_node src/trajectory_executor_node.cpp)
target_link_libraries(trajectory_executor_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/manual_trajectory_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslib REQUIRED)
  include_directories(${roslib_INCLUDE_DIRS})

  catkin_add_gtest(${PROJECT_NAME}-test-quintic-time-scaling test/test_quintic_time_scaling.cpp)
  if(TARGET ${PROJECT_NAME}-test-quintic-time-scaling)
    target_link_libraries(${PROJECT_NAME}-test-quintic-time-scaling arm_trajectory ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-test-joint-space-trajectory test/test_joint_space_trajectory.cpp)
  if(TARGET ${PROJECT_NAME}-test-joint-space-trajectory)
    target_link_libraries(${PROJECT_NAME}-test-joint-space-trajectory arm_trajectory ${catkin_LIBRARIES} ${roslib_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-test-cartesian-trajectory test/test_cartesian_trajectory.cpp)
  if(TARGET ${PROJECT_NAME}-test-cartesian-trajectory)
    target_link_libraries(${PROJECT_NAME}-test-cartesian-trajectory arm_trajectory ${catkin_LIBRARIES} ${roslib_LIBRARIES})
  endif()

  find_package(rostest REQUIRED)
  add_rostest(test/trajectory_execution.test)
endif()
