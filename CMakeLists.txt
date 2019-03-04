cmake_minimum_required(VERSION 2.8.3)
project(adaptive_grasp_controller)
add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rviz
  rviz_visual_tools
  message_generation
  actionlib_msgs
  geometry_msgs
  actionlib
  sensor_msgs
  tf
  tf_conversions
  moveit_msgs
  moveit_ros_planning 
  moveit_ros_planning_interface
  moveit_visual_tools
  interactive_markers 
  visualization_msgs
  moveit_core
)

add_service_files(FILES
  adaptive_controller.srv
  adaptive_control.srv
  slerp_control.srv
  hand_control.srv
  pose_control.srv
  joint_control.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  tf
  geometry_msgs
)

find_package(SDFormat REQUIRED)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs moveit_ros_planning_interface
  LIBRARIES moveit_ros_planning_interface sdformat
)

set(OBJECTS_SOURCE_FILES
  src/SlerpControl.cpp 
  src/HandControl.cpp 
  src/PoseControl.cpp 
  src/AdaptiveControl.cpp 
  src/JointControl.cpp
  src/PandaSoftHandClient.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS} /usr/include/eigen3 ${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})

add_executable(adaptive_grasp_controller src/AdaptiveGraspController.cpp)
target_link_libraries(adaptive_grasp_controller ${catkin_LIBRARIES} ${ROS_LIBRARIES})

add_executable(main_server src/main_server.cpp ${OBJECTS_SOURCE_FILES})
target_link_libraries(main_server ${catkin_LIBRARIES} ${ROS_LIBRARIES})

add_executable(publish_touch_demo_node src/PublishTouchDemo.cpp)
target_link_libraries(publish_touch_demo_node ${catkin_LIBRARIES} ${ROS_LIBRARIES})

add_executable(sdf_to_planning_scene_node src/SdfToPlanningScene.cpp)
target_link_libraries(sdf_to_planning_scene_node ${catkin_LIBRARIES} ${ROS_LIBRARIES} ${SDFormat_LIBRARIES}) 