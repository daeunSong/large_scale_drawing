#include "drawing_input_range.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <ros/package.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>


class DrawingMoveit{
  public:
    // Move Group parameters
    const double backward = 0.06;
    const double jump_threshold = 0.0; // 0.0
    const double eef_step = 0.001; // 0.001
    std::string ee_link;
    moveit::planning_interface::MoveGroupInterface *move_group;
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // to draw lines in rviz
    geometry_msgs::Point color;
    ros::Publisher drawing_line_pub;
    ros::Publisher drawing_color_pub;

    DrawingMoveit(ros::NodeHandle &nh, std::string planning_group, std::string planner_id, std::string ee_link_, std::string reference_frame);

    void addScene(int j);
    void moveInitPose();
    geometry_msgs::PoseStamped getCurrentPose();
    void drawStrokes(ros::NodeHandle &nh, DrawingInput &drawing_coor, char color_, int range_num);
};
