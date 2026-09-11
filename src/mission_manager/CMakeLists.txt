cmake_minimum_required(VERSION 3.0.2)
project(mission_manager)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  move_base_msgs
  multifloor_manager
  nav_msgs
  rospy
  stair_supervisor
  std_msgs
  std_srvs
)

catkin_python_setup()

add_action_files(
  FILES
  Mission.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs message_runtime move_base_msgs multifloor_manager nav_msgs rospy stair_supervisor std_msgs std_srvs
)

  catkin_install_python(
  PROGRAMS scripts/mission_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_fixture_discovery.py)
  catkin_add_nosetests(test/test_route_planner.py)
  catkin_add_nosetests(test/test_scan_recorder.py)
  catkin_add_nosetests(test/test_subprocess_executor.py)
  catkin_add_nosetests(test/test_fsm.py)
  catkin_add_nosetests(test/test_navigation_executor.py)
  catkin_add_nosetests(test/test_ros_segments.py)
  catkin_add_nosetests(test/test_mission_orchestrator.py)
  catkin_add_nosetests(test/test_runtime_gating.py)
  set(_mission_python_executable "${PYTHON_EXECUTABLE}")
  set(PYTHON_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/test/rostest_python.sh")
  add_rostest(
    test/process_boundaries.test
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  add_rostest(
    test/scan_recorder_ros.test
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  add_rostest(
    test/mission_system.test
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  add_rostest(
    test/mission_5f_rf_synthetic.test
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  set(PYTHON_EXECUTABLE "${_mission_python_executable}")
endif()
