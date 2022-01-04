cmake_minimum_required(VERSION 3.5)
project(large_scale_drawing)

# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  iiwa_ros
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  actionlib
  move_base_msgs #below
  geometry_msgs
  std_msgs
  roslib
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
)

include_directories(iiwa/include ${catkin_INCLUDE_DIRS})

ADD_LIBRARY(drawingInput
  iiwa/src/drawing_input_range.cpp
)
ADD_LIBRARY(drawingMoveit
  iiwa/src/moveit/drawing_moveit.cpp
)
ADD_LIBRARY(KDTree
  iiwa/src/KDTree.cpp
)

#add_message_files(
#  FILES
#  drawingRange.msg
#  minmaxY.msg
#  drawingPose.msg
#)
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs 
#)

catkin_package()

# sim
#add_executable(drawing_moveit iiwa/src/moveit/drawing_moveit.cpp)
#add_dependencies(drawing_moveit iiwa_msgs_generate_messages_cpp)
#target_link_libraries(drawing_moveit drawingInput KDTree ${catkin_LIBRARIES}) #

add_executable(drawing_manager drawing_manager.cpp)
add_dependencies(drawing_manager iiwa_msgs_generate_messages_cpp)
target_link_libraries(drawing_manager drawingInput drawingMoveit KDTree ${catkin_LIBRARIES}) #

add_executable(line_publisher iiwa/src/moveit/line_publisher.cpp)
add_dependencies(line_publisher iiwa_msgs_generate_messages_cpp)
target_link_libraries(line_publisher ${catkin_LIBRARIES})

# real
add_executable(drawing_iiwa iiwa/src/drawing_iiwa.cpp)
add_dependencies(drawing_iiwa iiwa_msgs_generate_messages_cpp)
target_link_libraries(drawing_iiwa drawingInput KDTree ${catkin_LIBRARIES}) #


