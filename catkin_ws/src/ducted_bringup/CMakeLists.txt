cmake_minimum_required(VERSION 3.0.2)
project(ducted_bringup)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  ducted_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs ducted_msgs mavros_msgs nav_msgs rospy sensor_msgs std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/base_health_monitor.py
    scripts/fastlio_odometry_adapter.py
    scripts/world_tf_owner.py
    scripts/external_odometry_relay.py
    scripts/startup_check.py
    scripts/record_topics.py
    scripts/save_map.py
    scripts/terrain_height_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_base_health.py)
  catkin_add_nosetests(test/test_fastlio_odometry_adapter.py)
  catkin_add_nosetests(test/test_odometry_runtime.py)
  catkin_add_nosetests(test/test_external_odometry_relay.py)
  catkin_add_nosetests(test/test_observability.py)
  catkin_add_nosetests(test/test_save_map.py)
  catkin_add_nosetests(test/test_world_frames.py)
  catkin_add_nosetests(test/test_world_tf_runtime.py)
  catkin_add_nosetests(test/test_mapping_contract.py)
  catkin_add_nosetests(test/test_relocalization_contract.py)
  catkin_add_nosetests(test/test_scaffold_contract.py)
  catkin_add_nosetests(test/test_terrain_height.py)
  catkin_add_nosetests(test/test_terrain_runtime.py)
  catkin_add_nosetests(test/test_forward_profile.py)
endif()
