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
)

add_service_files(
    FILES
    apriltag_detect.srv
    object_detect.srv
)

add_action_files(
    DIRECTORY
    FILES
    WaypointMove.action
    ApriltagSearch.action
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

add_executable(node_B_1 src/node_B_1.cpp)
add_dependencies(node_B_1 assignment2_generate_messages_cpp)
target_link_libraries(node_B_1 ${catkin_LIBRARIES})

add_executable(node_C src/node_C.cpp)
add_dependencies(node_C assignment2_generate_messages_cpp)
target_link_libraries(node_C ${catkin_LIBRARIES})

add_executable(apriltag_locator src/apriltag_locator.cpp)
add_dependencies(apriltag_locator assignment2_generate_messages_cpp)
target_link_libraries(apriltag_locator ${catkin_LIBRARIES})

add_executable(apriltag_locator_service src/apriltag_detect_service.cpp)
add_dependencies(apriltag_locator_service assignment2_generate_messages_cpp)
target_link_libraries(apriltag_locator_service ${catkin_LIBRARIES})

add_executable(node_C_tester src/node_C_tester.cpp)
add_dependencies(node_C_tester assignment2_generate_messages_cpp)
target_link_libraries(node_C_tester ${catkin_LIBRARIES})
