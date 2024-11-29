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
)

add_action_files(
    DIRECTORY
    FILES
    WaypointMove.action
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

add_executable(coordinator src/coordinator.cpp)
add_dependencies(coordinator assignment1_generate_messages_cpp)
target_link_libraries(coordinator ${catkin_LIBRARIES})

add_executable(MovementHandler src/movement_handler.cpp)
add_dependencies(MovementHandler assignment1_generate_messages_cpp)
target_link_libraries(MovementHandler ${catkin_LIBRARIES})

add_executable(look_downer src/look_down.cpp)
target_link_libraries(look_downer ${catkin_LIBRARIES})
