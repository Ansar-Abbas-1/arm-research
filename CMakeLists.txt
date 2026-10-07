cmake_minimum_required(VERSION 3.0.2)
project(arm_servo)

find_package(catkin REQUIRED COMPONENTS
  roscpp arm_kinematics arm_trajectory arm_perception urdf sensor_msgs std_msgs std_srvs geometry_msgs tf2_ros tf2_eigen roslaunch)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arm_servo
  CATKIN_DEPENDS roscpp arm_kinematics arm_trajectory arm_perception urdf sensor_msgs std_msgs std_srvs geometry_msgs tf2_ros tf2_eigen
  DEPENDS EIGEN3)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(arm_servo src/servo_law.cpp)
add_dependencies(arm_servo ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_servo ${catkin_LIBRARIES})

add_executable(visual_servo_node src/visual_servo_node.cpp)
add_dependencies(visual_servo_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(visual_servo_node arm_servo ${catkin_LIBRARIES})

roslaunch_add_file_check(launch)
install(TARGETS arm_servo visual_servo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(roslib REQUIRED)
  include_directories(${roslib_INCLUDE_DIRS})
  catkin_add_gtest(${PROJECT_NAME}-test-servo-law test/test_servo_law.cpp)
  if(TARGET ${PROJECT_NAME}-test-servo-law)
    target_link_libraries(${PROJECT_NAME}-test-servo-law arm_servo ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(${PROJECT_NAME}-test-closed-loop test/test_closed_loop.cpp)
  if(TARGET ${PROJECT_NAME}-test-closed-loop)
    target_link_libraries(${PROJECT_NAME}-test-closed-loop arm_servo ${catkin_LIBRARIES} ${roslib_LIBRARIES})
  endif()
endif()
