cmake_minimum_required(VERSION 3.0.2)
project(ducted_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_scaffold_contract.py)
endif()
