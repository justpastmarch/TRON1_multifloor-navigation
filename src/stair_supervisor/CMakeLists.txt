cmake_minimum_required(VERSION 3.0.2)
project(stair_supervisor)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  geometry_msgs
  message_generation
  nav_msgs
  rosbag
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

catkin_python_setup()

add_action_files(
  FILES
  StairTraversal.action
)

add_message_files(
  FILES
  SupervisorState.msg
)

add_service_files(
  FILES
  ValidateStairAdmission.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs cv_bridge geometry_msgs message_runtime nav_msgs rosbag rospy sensor_msgs std_msgs std_srvs visualization_msgs
)

catkin_install_python(
  PROGRAMS
  scripts/replay_lidar_tracking.py
  scripts/profile_lidar_stream.py
  scripts/calibrate_yaw_rate.py
  scripts/check_lidar_config.py
  scripts/stair_entry_test.py
  scripts/stair_supervisor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES requirements-lidar.txt DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/stair_python.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  option(STAIR_LIDAR_TESTS "Run optional numerical LiDAR tests in configured Python runtime" OFF)
  if(STAIR_LIDAR_TESTS)
    catkin_add_nosetests(test/test_lidar_control.py)
    catkin_add_nosetests(test/test_lidar_ros_adapter.py)
    catkin_add_nosetests(test/test_lidar_process.py)
    catkin_add_nosetests(test/test_lidar_phase_test.py)
    catkin_add_nosetests(test/test_stair_repair.py)
    catkin_add_nosetests(test/test_stair_readiness.py)
  endif()
  catkin_add_nosetests(test/test_fixture_discovery.py)
  catkin_add_nosetests(test/test_configuration_transition.py)
  catkin_add_nosetests(test/test_stair_evidence.py)
  catkin_add_nosetests(test/test_bag_replay.py)
  catkin_add_nosetests(test/test_state_machine_dashboard.py)
  catkin_add_nosetests(test/test_supervisor.py)
  catkin_add_nosetests(test/test_supervisor_safety.py)
  catkin_add_nosetests(test/robot_transport)
  set(_stair_python_executable "${PYTHON_EXECUTABLE}")
  set(PYTHON_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/test/rostest_python.sh")
  add_rostest(
    test/stair_supervisor_node.test
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  set(PYTHON_EXECUTABLE "${_stair_python_executable}")
endif()
