cmake_minimum_required(VERSION 3.0.2)
project(multifloor_manager)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  apriltag_ros
  geometry_msgs
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

catkin_python_setup()

add_action_files(
  FILES
  FloorTransition.action
)

add_message_files(
  FILES
  FloorState.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs apriltag_ros geometry_msgs message_runtime nav_msgs rospy sensor_msgs std_msgs std_srvs tf2_ros
)

catkin_install_python(
  PROGRAMS
    scripts/multifloor_manager_node.py
    scripts/rviz_tf_reconnect.py
    scripts/camera_info_stamp_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS scripts/wait_for_tf_exec.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  get_filename_component(TRON1_WORKSPACE_ROOT "${CMAKE_SOURCE_DIR}" DIRECTORY)
  catkin_add_nosetests(test/test_fixture_discovery.py)
  catkin_add_nosetests("${TRON1_WORKSPACE_ROOT}/test/test_tag_evidence.py")
  catkin_add_nosetests("${TRON1_WORKSPACE_ROOT}/test/test_map_evidence.py")
  catkin_add_nosetests("${TRON1_WORKSPACE_ROOT}/test/test_site_configuration.py")
  catkin_add_nosetests("${TRON1_WORKSPACE_ROOT}/test/test_stair_endpoint_configuration.py")
  catkin_add_nosetests(test/test_map_loader.py)
  catkin_add_nosetests(test/test_ros_services.py)
  catkin_add_nosetests(test/test_readiness.py)
  catkin_add_nosetests(test/test_startup_matching.py)
  catkin_add_nosetests(test/test_transitions.py)
  catkin_add_nosetests(test/test_policy_final_gate.py)
  set(_multifloor_python_executable "${PYTHON_EXECUTABLE}")
  set(PYTHON_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/test/rostest_python.sh")
  add_rostest(
    test/floor_transition_ros.test
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  add_rostest(
    test/floor_transition_policy_blocked_ros.test
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  add_rostest(
    test/floor_transition_cancel_ros.test
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  add_rostest(
    test/floor_transition_late_map_ros.test
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  add_rostest(
    test/floor_transition_epoch_ros.test
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  add_rostest(
    test/floor_transition_delayed_pose_ros.test
    DEPENDENCIES ${PROJECT_NAME}_generate_messages_py
  )
  add_rostest(test/startup_localization.test DEPENDENCIES ${PROJECT_NAME}_generate_messages_py)
  set(PYTHON_EXECUTABLE "${_multifloor_python_executable}")
endif()
