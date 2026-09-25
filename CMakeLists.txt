cmake_minimum_required(VERSION 3.0.2)
project(arm_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arm_kinematics
  CATKIN_DEPENDS roscpp urdf geometry_msgs sensor_msgs tf2 tf2_ros tf2_eigen
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(arm_kinematics
  src/forward_kinematics.cpp
)
target_link_libraries(arm_kinematics
  ${catkin_LIBRARIES}
)

add_executable(fk_node src/fk_node.cpp)
target_link_libraries(fk_node
  arm_kinematics
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslib REQUIRED)
  include_directories(${roslib_INCLUDE_DIRS})

  catkin_add_gtest(${PROJECT_NAME}-test-forward-kinematics test/test_forward_kinematics.cpp)
  if(TARGET ${PROJECT_NAME}-test-forward-kinematics)
    target_link_libraries(${PROJECT_NAME}-test-forward-kinematics arm_kinematics ${catkin_LIBRARIES} ${roslib_LIBRARIES})
  endif()

  find_package(rostest REQUIRED)
  add_rostest(test/fk_validation.test)
endif()
