#include "drawing_moveit.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include "boost/shared_ptr.hpp"
#include <visualization_msgs/Marker.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define Stroke std::vector<geometry_msgs::Pose>

class DrawingManager {
  public:
    DrawingManager(ros::NodeHandle* nh);

    std::vector<DrawingInput> drawings;
    std_msgs::Float64MultiArray ranges;
    int range_num;

    std_msgs::String iiwa_state;
    std::string ridgeback_state;

    visualization_msgs::Marker marker;
    visualization_msgs::Marker target_range_marker;

    ros::Publisher ir_pub;
    ros::Publisher marker_pub;

    // MoveGroup
    std::string move_group_name;
    std::string ee_link;
    std::string planner_id;
    std::string reference_frame;

    // iiwa
    geometry_msgs::Pose init_drawing_pose;

    // problem
    std::string wall_file_name, drawing_file_name;
    std::vector<double> wall_pose;
    std::vector<std::string> colors;

    void visualizeStrokes(std::vector<Stroke> &strokes, char color);
    void publishState(std::string state);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber ir_sub_;

    void initSubscriber();
    void initPublisher();
    void initMarker();
    void initMoveGroup();

    // MoveGroup
    const std::string PLANNING_GROUP = "manipulator";
    const std::string EE_LINK = "iiwa_link_ee";
    const std::string PLANNER_ID = "geometric::RRTConnect";
    const std::string REFERENCE_FRAME = "base_link";

    void stateCallback(const std_msgs::String::ConstPtr& msg);
};
