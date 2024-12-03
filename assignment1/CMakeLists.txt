cmake_minimum_required(VERSION 3.0.2)
project(assignment1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  actionlib
  apriltag_ros
  geometry_msgs
)

add_service_files(
    FILES
    map_service.srv
    map_waypoints.srv
    apriltag_detect.srv
)

add_action_files(
    DIRECTORY
    FILES
    WaypointMove.action
    ApriltagSearch.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
)
 
catkin_package(
# INCLUDE_DIRS include
# LIBRARIES intro_services
  CATKIN_DEPENDS roscpp std_msgs
# DEPENDS system_lib
)

catkin_package( 
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  message_runtime
  actionlib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_listener src/map_listener.cpp)
target_link_libraries(map_listener ${catkin_LIBRARIES})

add_executable(waypoints_finder src/waypoints_finder.cpp)
target_link_libraries(waypoints_finder ${catkin_LIBRARIES})

add_executable(first_move_test src/first_move_test.cpp)
target_link_libraries(first_move_test ${catkin_LIBRARIES})

add_executable(map_service src/map_service.cpp)
target_link_libraries(map_service ${catkin_LIBRARIES})

add_executable(map_divider src/map_divider.cpp)
target_link_libraries(map_divider ${catkin_LIBRARIES})

add_executable(map_waypoints_service src/map_waypoints_service.cpp)
add_dependencies(map_waypoints_service assignment1_generate_messages_cpp)
target_link_libraries(map_waypoints_service ${catkin_LIBRARIES})

add_executable(apriltag_locator src/apriltag_locator.cpp)
add_dependencies(apriltag_locator assignment1_generate_messages_cpp)
target_link_libraries(apriltag_locator ${catkin_LIBRARIES})

add_executable(movement_handler src/movement_handler.cpp)
add_dependencies(movement_handler assignment1_generate_messages_cpp)
target_link_libraries(movement_handler ${catkin_LIBRARIES})

# add_executable(look_downer src/look_down.cpp)
# target_link_libraries(look_downer ${catkin_LIBRARIES})

add_executable(node_A src/node_A.cpp)
add_dependencies(node_A assignment1_generate_messages_cpp)
target_link_libraries(node_A ${catkin_LIBRARIES})

add_executable(node_B src/node_B.cpp)
add_dependencies(node_B assignment1_generate_messages_cpp)
target_link_libraries(node_B ${catkin_LIBRARIES})