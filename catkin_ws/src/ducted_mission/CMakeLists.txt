cmake_minimum_required(VERSION 3.0.2)
project(ducted_mission)

find_package(catkin REQUIRED COMPONENTS
  ducted_msgs
  ducted_navigation
  geometry_msgs
  mavros_msgs
  sensor_msgs
  message_generation
  nav_msgs
  rospy
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
)

catkin_python_setup()

add_message_files(FILES MissionStatus.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS
  ducted_msgs ducted_navigation geometry_msgs mavros_msgs sensor_msgs message_runtime nav_msgs rospy
  std_msgs std_srvs tf2_geometry_msgs tf2_ros
)

catkin_install_python(PROGRAMS scripts/waypoint_mission.py scripts/automatic_flight.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_automatic.py)
  catkin_add_nosetests(test/test_ground_reference.py)
  catkin_add_nosetests(test/test_adapter.py)
  catkin_add_nosetests(test/test_mission.py)
  catkin_add_nosetests(test/test_runtime.py)
  catkin_add_nosetests(test/test_contract.py)
endif()
