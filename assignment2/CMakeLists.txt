cmake_minimum_required(VERSION 3.0.2)
project(assignment2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  actionlib
  apriltag_ros
  geometry_msgs
  tiago_iaslab_simulation
  moveit_core
  moveit_ros_planning_interface
  shape_msgs
  gazebo_ros_link_attacher
  control_msgs
  image_geometry
)

add_service_files(
    FILES
    apriltag_detect.srv
)

add_action_files(
    DIRECTORY
    FILES
    ObjectMove.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  message_runtime
  actionlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(movement
  src/movement.cpp
)
target_link_libraries(movement
  ${catkin_LIBRARIES}
)

add_executable(node_A src/node_A.cpp)
add_dependencies(node_A assignment2_generate_messages_cpp)
target_link_libraries(node_A
  movement
  ${catkin_LIBRARIES}
)

add_executable(node_B src/node_B.cpp)
add_dependencies(node_B assignment2_generate_messages_cpp)
target_link_libraries(node_B ${catkin_LIBRARIES})

add_executable(node_C src/node_C.cpp)
add_dependencies(node_C assignment2_generate_messages_cpp)
target_link_libraries(node_C ${catkin_LIBRARIES})

