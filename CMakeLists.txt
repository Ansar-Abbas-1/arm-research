cmake_minimum_required(VERSION 3.0.2)
project(arm_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  arm_kinematics
  arm_trajectory
  urdf
  geometry_msgs
  sensor_msgs
  control_msgs
  actionlib
  actionlib_msgs
  visualization_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arm_teleop
  CATKIN_DEPENDS roscpp arm_kinematics arm_trajectory urdf geometry_msgs sensor_msgs control_msgs actionlib actionlib_msgs visualization_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(arm_teleop
  src/joy_target_mapper.cpp
  src/resolved_rate.cpp
)
add_dependencies(arm_teleop ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_teleop
  ${catkin_LIBRARIES}
)

add_executable(teleop_node src/teleop_node.cpp)
add_dependencies(teleop_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_node
  arm_teleop
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/manual_teleop_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslib REQUIRED)
  include_directories(${roslib_INCLUDE_DIRS})

  catkin_add_gtest(${PROJECT_NAME}-test-joy-target-mapper test/test_joy_target_mapper.cpp)
  if(TARGET ${PROJECT_NAME}-test-joy-target-mapper)
    target_link_libraries(${PROJECT_NAME}-test-joy-target-mapper arm_teleop ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-test-resolved-rate test/test_resolved_rate.cpp)
  if(TARGET ${PROJECT_NAME}-test-resolved-rate)
    target_link_libraries(${PROJECT_NAME}-test-resolved-rate arm_teleop ${catkin_LIBRARIES} ${roslib_LIBRARIES})
  endif()

  find_package(rostest REQUIRED)
  add_rostest(test/teleop_modes.test)
  add_rostest(test/teleop_nullspace.test)
endif()
