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
  move_base_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

# sim
add_executable(drawing_moveit iiwa/src/moveit/drawing_moveit.cpp)
add_dependencies(drawing_moveit iiwa_msgs_generate_messages_cpp)
target_link_libraries(drawing_moveit ${catkin_LIBRARIES})

# real
add_executable(drawing_iiwa iiwa/src/drawing_iiwa.cpp)
add_dependencies(drawing_iiwa iiwa_msgs_generate_messages_cpp)
target_link_libraries(drawing_iiwa ${catkin_LIBRARIES})


