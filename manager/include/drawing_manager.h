#include "drawing_iiwa.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
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

    std_msgs::Int32 iiwa_state;
    int ridgeback_state;

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
    geometry_msgs::Pose wall_;
    geometry_msgs::Pose iiwa_;
    std::vector<std::string> colors;

    void visualizeStrokes(std::vector<Stroke> &strokes);
    void publishState(int state);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber ir_sub_;
    ros::Subscriber iiwa_pose_sub_;
    ros::Subscriber wall_sub_;

    void initSubscriber();
    void initPublisher();
    void initMarker();
    void initMoveGroup();

    // MoveGroup
    const std::string PLANNING_GROUP = "manipulator";
    const std::string EE_LINK = "tool_link_ee";
    const std::string PLANNER_ID = "geometric::RRTConnect";
    const std::string REFERENCE_FRAME = "base_link";

    void stateCallback(const std_msgs::Int32::ConstPtr& msg);
    void wallCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void iiwaCallback(const geometry_msgs::Pose::ConstPtr& msg);
};
