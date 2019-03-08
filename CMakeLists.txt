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
  interactive_markers 
  visualization_msgs
  moveit_core
)

add_service_files(FILES
  adaptive_controller.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  tf
)

find_package(SDFormat REQUIRED)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs moveit_ros_planning_interface message_runtime
  LIBRARIES moveit_ros_planning_interface sdformat
)

include_directories(include ${catkin_INCLUDE_DIRS} /usr/include/eigen3 ${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})

add_executable(adaptive_grasp_controller src/AdaptiveGraspController.cpp)
add_dependencies(adaptive_grasp_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(adaptive_grasp_controller ${catkin_LIBRARIES} ${ROS_LIBRARIES}) 

add_executable(publish_touch_demo_node src/PublishTouchDemo.cpp)
target_link_libraries(publish_touch_demo_node ${catkin_LIBRARIES} ${ROS_LIBRARIES})

add_executable(sdf_to_planning_scene_node src/SdfToPlanningScene.cpp)
target_link_libraries(sdf_to_planning_scene_node ${catkin_LIBRARIES} ${ROS_LIBRARIES} ${SDFormat_LIBRARIES}) 