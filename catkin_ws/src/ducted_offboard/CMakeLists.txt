cmake_minimum_required(VERSION 3.0.2)
project(ducted_offboard)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  roscpp
  std_msgs
)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

add_message_files(FILES OffboardStatus.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ducted_offboard_core ducted_offboard_mission
  CATKIN_DEPENDS geometry_msgs mavros_msgs message_runtime nav_msgs roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(ducted_offboard_core src/types.cpp src/task_machine.cpp src/execution.cpp)
target_link_libraries(ducted_offboard_core ${catkin_LIBRARIES})
add_library(ducted_offboard_mission src/mission_loader.cpp)
target_link_libraries(ducted_offboard_mission ducted_offboard_core
  ${catkin_LIBRARIES} yaml-cpp)

add_executable(offboard_controller_node src/offboard_node.cpp)
add_dependencies(offboard_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(offboard_controller_node ducted_offboard_mission
  ducted_offboard_core ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

install(TARGETS ducted_offboard_core ducted_offboard_mission offboard_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  # Frozen implementations are available exclusively to offline regression tests.
  add_library(ducted_offboard_legacy STATIC src/core.cpp src/planner_core.cpp)
  target_link_libraries(ducted_offboard_legacy ducted_offboard_core)
  catkin_add_gtest(test_split_runtime test/test_split_runtime.cpp)
  if(TARGET test_split_runtime)
    target_link_libraries(test_split_runtime ducted_offboard_core ${catkin_LIBRARIES})
  endif()
  add_executable(offboard_controller_test_node EXCLUDE_FROM_ALL src/offboard_node.cpp)
  target_compile_definitions(offboard_controller_test_node PRIVATE DUCTED_OFFBOARD_TEST_HOOKS=1)
  add_dependencies(offboard_controller_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(offboard_controller_test_node ducted_offboard_mission ducted_offboard_core
    ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)
  catkin_add_gtest(test_ego_core test/test_ego_core.cpp)
  if(TARGET test_ego_core)
    target_link_libraries(test_ego_core ducted_offboard_legacy ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_offboard_core test/test_core.cpp)
  if(TARGET test_offboard_core)
    add_dependencies(test_offboard_core ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(test_offboard_core ducted_offboard_legacy ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_mission_loader test/test_mission_loader.cpp)
  if(TARGET test_mission_loader)
    add_dependencies(test_mission_loader ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(test_mission_loader ducted_offboard_mission
      ducted_offboard_core ${catkin_LIBRARIES} yaml-cpp)
  endif()
endif()
