#include "drawing_input_range.h"
#include <ros/ros.h>
#include <ros/package.h>

#include <actionlib/client/simple_action_client.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>
#include <iiwa_msgs/MoveToCartesianPoseAction.h>
#include <iiwa_msgs/MoveAlongSplineAction.h>
#include <iiwa_msgs/SetPTPJointSpeedLimits.h>
#include <iiwa_msgs/SetPTPCartesianSpeedLimits.h>
#include <iiwa_msgs/SetSmartServoLinSpeedLimits.h>
#include <iiwa_msgs/SetEndpointFrame.h>
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/cartesian_pose_linear.hpp>
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/service/control_mode.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>
#include <iiwa_ros/iiwa_ros.hpp>
#include <iiwa_ros/conversions.hpp>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define BACKWARD 0.05

class DrawingIIWA{
  public:
    DrawingIIWA(ros::NodeHandle &nh, std::string ee_link_, std::string reference_frame);

    iiwa_ros::command::CartesianPoseLinear iiwa_pose_command;
    iiwa_ros::state::CartesianPose iiwa_pose_state;

    // for Cartesian Impedance Control
    iiwa_msgs::CartesianQuantity cartesian_stiffness;
    iiwa_msgs::CartesianQuantity cartesian_damping;

    iiwa_ros::service::ControlModeService iiwa_control_mode;
    iiwa_ros::service::TimeToDestinationService iiwa_time_destination;
    actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction> jointPositionClient;
//    actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction> cartesianPositionClient;
    actionlib::SimpleActionClient<iiwa_msgs::MoveAlongSplineAction> splineMotionClient;

//    actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction> && jointPositionClient =
//            actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction>("/iiwa/action/move_to_joint_position", true);
//    actionlib::SimpleActionClient<iiwa_msgs::MoveAlongSplineAction> && splineMotionClient =
//            actionlib::SimpleActionClient<iiwa_msgs::MoveAlongSplineAction>("/iiwa/action/move_along_spline", true);

    iiwa_msgs::CartesianPose init_pose;

    int init(ros::NodeHandle &nh, std::string ee_link_);
    int moveInitPose();
    int moveTransportPose();
    iiwa_msgs::CartesianPose getCurrentPose();
    geometry_msgs::Point detectWall(ros::NodeHandle &nh);
    void drawStrokes(ros::NodeHandle &nh, DrawingInput &drawing_strokes, int range_num, int stroke_num);

    void sleepForMotion(iiwa_ros::service::TimeToDestinationService& iiwa, const double maxSleepTime);
    iiwa_msgs::SplineSegment getSplineSegment (geometry_msgs::Pose waypoint_pose, int type);

    bool setPTPJointSpeedLimits(ros::NodeHandle& nh);
    bool setPTPCartesianSpeedLimits(ros::NodeHandle& nh);
    bool setSmartServoLinSpeedLimits(ros::NodeHandle& nh);
    bool setEndpointFrame(ros::NodeHandle& nh, std::string frameId);
};
