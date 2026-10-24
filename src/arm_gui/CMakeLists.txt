cmake_minimum_required(VERSION 3.0.2)
project(arm_gui)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  std_msgs
  std_srvs
  diagnostic_msgs
  sensor_msgs
  pluginlib
  message_generation
)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Test)

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 14)

add_service_files(FILES MissionCommand.srv PresetCommand.srv SetLoadTarget.srv)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs std_srvs diagnostic_msgs sensor_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

# The panel is a plugin library; the tests link the same library.
add_library(arm_gui src/arm_control_panel.cpp include/arm_gui/arm_control_panel.h include/arm_gui/panel_logic.h
            src/mission_panel.cpp include/arm_gui/mission_panel.h)
target_link_libraries(arm_gui ${catkin_LIBRARIES} Qt5::Widgets)
add_dependencies(arm_gui ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

install(TARGETS arm_gui LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_panel_logic test/test_panel_logic.cpp)
  # The panel test needs a ROS master (rostest starts one) and runs Qt headless (QT_QPA_PLATFORM=offscreen).
  add_rostest_gtest(test_arm_control_panel test/arm_control_panel.test test/test_arm_control_panel.cpp)
  target_link_libraries(test_arm_control_panel arm_gui ${catkin_LIBRARIES} Qt5::Widgets Qt5::Test)
  add_rostest_gtest(test_mission_panel test/mission_panel.test test/test_mission_panel.cpp)
  target_link_libraries(test_mission_panel arm_gui ${catkin_LIBRARIES} Qt5::Widgets Qt5::Test)
endif()

# Debug tool, see tools/panel_probe.cpp (not installed).
add_executable(panel_probe tools/panel_probe.cpp)
target_link_libraries(panel_probe arm_gui ${catkin_LIBRARIES} Qt5::Widgets)
