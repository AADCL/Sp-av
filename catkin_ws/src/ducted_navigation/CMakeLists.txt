cmake_minimum_required(VERSION 3.0.2)
project(ducted_navigation)

find_package(catkin REQUIRED COMPONENTS
  ducted_msgs
  ducted_planning
  geometry_msgs
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_python_setup()

add_message_files(FILES NavigationStatus.msg)
add_service_files(FILES Navigate.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS ducted_msgs geometry_msgs message_runtime nav_msgs rospy sensor_msgs std_msgs tf2_ros
)

catkin_install_python(
  PROGRAMS scripts/ducted_navigation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_ego_worker.py)
  catkin_add_nosetests(test/test_cloud_input.py)
  catkin_add_nosetests(test/test_failure_recorder.py)
  catkin_add_nosetests(test/test_planner_context.py)
  catkin_add_nosetests(test/test_relative_height.py)
  catkin_add_nosetests(test/test_trajectory_guard.py)
  catkin_add_nosetests(test/test_planner_core.py)
  catkin_add_nosetests(test/test_runtime.py)
  catkin_add_nosetests(test/test_node_runtime.py)
  catkin_add_nosetests(test/test_contract.py)
endif()
