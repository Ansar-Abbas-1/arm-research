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
  roslaunch
  message_generation
)
find_package(Eigen3 REQUIRED)

roslaunch_add_file_check(launch)

add_service_files(
  FILES
  SolveIk.srv
  SolvePositionPitchIk.srv
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arm_kinematics
  CATKIN_DEPENDS roscpp urdf geometry_msgs sensor_msgs tf2 tf2_ros tf2_eigen message_runtime
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(arm_kinematics
  src/forward_kinematics.cpp
  src/jacobian.cpp
  src/inverse_kinematics.cpp
  src/workspace_sampler.cpp
  src/task_space_math.cpp
  src/resolved_rate.cpp
  src/position_pitch_ik.cpp
  src/base_collision.cpp
)
target_link_libraries(arm_kinematics
  ${catkin_LIBRARIES}
)

add_executable(fk_node src/fk_node.cpp)
target_link_libraries(fk_node
  arm_kinematics
  ${catkin_LIBRARIES}
)

add_executable(workspace_mapper_node src/workspace_mapper_node.cpp)
target_link_libraries(workspace_mapper_node
  arm_kinematics
  ${catkin_LIBRARIES}
)

add_executable(resolved_rate_demo_node src/resolved_rate_demo_node.cpp)
target_link_libraries(resolved_rate_demo_node
  arm_kinematics
  ${catkin_LIBRARIES}
)

add_executable(ik_node src/ik_node.cpp)
add_dependencies(ik_node ${PROJECT_NAME}_gencpp)
target_link_libraries(ik_node
  arm_kinematics
  ${catkin_LIBRARIES}
)

add_executable(position_pitch_ik_node src/position_pitch_ik_node.cpp)
add_dependencies(position_pitch_ik_node ${PROJECT_NAME}_gencpp)
target_link_libraries(position_pitch_ik_node
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

  catkin_add_gtest(${PROJECT_NAME}-test-jacobian test/test_jacobian.cpp)
  if(TARGET ${PROJECT_NAME}-test-jacobian)
    target_link_libraries(${PROJECT_NAME}-test-jacobian arm_kinematics ${catkin_LIBRARIES} ${roslib_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-test-inverse-kinematics test/test_inverse_kinematics.cpp)
  if(TARGET ${PROJECT_NAME}-test-inverse-kinematics)
    target_link_libraries(${PROJECT_NAME}-test-inverse-kinematics arm_kinematics ${catkin_LIBRARIES} ${roslib_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-test-workspace-sampler test/test_workspace_sampler.cpp)
  if(TARGET ${PROJECT_NAME}-test-workspace-sampler)
    target_link_libraries(${PROJECT_NAME}-test-workspace-sampler arm_kinematics ${catkin_LIBRARIES} ${roslib_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-test-resolved-rate test/test_resolved_rate.cpp)
  if(TARGET ${PROJECT_NAME}-test-resolved-rate)
    target_link_libraries(${PROJECT_NAME}-test-resolved-rate arm_kinematics ${catkin_LIBRARIES} ${roslib_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-test-position-pitch-ik test/test_position_pitch_ik.cpp)
  if(TARGET ${PROJECT_NAME}-test-position-pitch-ik)
    target_link_libraries(${PROJECT_NAME}-test-position-pitch-ik arm_kinematics ${catkin_LIBRARIES} ${roslib_LIBRARIES})
  endif()

  catkin_add_gtest(${PROJECT_NAME}-test-base-collision test/test_base_collision.cpp)
  if(TARGET ${PROJECT_NAME}-test-base-collision)
    target_link_libraries(${PROJECT_NAME}-test-base-collision arm_kinematics ${catkin_LIBRARIES} ${roslib_LIBRARIES})
  endif()

  find_package(rostest REQUIRED)
  add_rostest(test/fk_validation.test)
  add_rostest(test/ik_validation.test)
endif()
