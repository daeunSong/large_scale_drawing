#include "drawing_input_vis.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
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

#define BACKWARD 0.06

using namespace std;
using moveit::planning_interface::MoveItErrorCode;

// Create MoveGroup
static const std::string PLANNING_GROUP = "manipulator";
static const std::string EE_LINK = "iiwa_link_ee";
/* RRTConnectkConfigDefault, RRTkConfigDefault, RRTstartkConfigDefault, TRRTkConfigDefault, ESTkConfigDefault
   SBLkConfigDefault, LBKPIECEkConfigDefault, BKPIECEkConfigDefault, PRMkConfigDefault, PRMstarkConfigDefault */
static const std::string PLANNER_ID = "RRTConnectkConfigDefault";
static const std::string REFERENCE_FRAME = "base_link";

bool sim;

int main (int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "CommandRobotMoveit");
  ros::NodeHandle nh("~");
  nh.param("sim", sim, true);

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();


  std::string movegroup_name, ee_link, planner_id, reference_frame;
  geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position, init_cartesian_position;
  std::vector<geometry_msgs::Pose> linear_path;
  geometry_msgs::Pose drawing_point;

  // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh.param<std::string>("move_group", movegroup_name, PLANNING_GROUP);
  nh.param<std::string>("ee_link", ee_link, EE_LINK);
  nh.param<std::string>("planner_id", planner_id, PLANNER_ID);
  nh.param<std::string>("reference_frame", reference_frame, REFERENCE_FRAME);

  // Create Move Group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0; // 0.0
  const double eef_step = 0.001; // 0.001
  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  const moveit::core::LinkModel* link_model =
    move_group.getCurrentState()->getLinkModel(ee_link);

  // Configure Move Group
  move_group.setPlanningTime(0.5);
  //move_group.setPlannerId(PLANNING_GROUP+"[RRTConnectkConfigDefault]");
  move_group.setPlannerId(planner_id);
  move_group.setEndEffectorLink(ee_link);
  move_group.setPoseReferenceFrame(reference_frame);
  //ROS_INFO("Planner ID: %s", move_group.getPlannerId().c_str());
  ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
  //ROS_INFO("Pose Reference Frame: %s", move_group.getPoseReferenceFrame().c_str());

  // Moveit Visualization Tool 
  moveit_visual_tools::MoveItVisualTools visual_tools("iiwa_link_0");
  // visual_tools.deleteAllMarkers();
  // visual_tools.trigger();

  string line;
  bool init = false;
  double x, y, z, fraction;
  vector<double> normal;
  MoveItErrorCode success_plan = MoveItErrorCode::FAILURE, motion_done = MoveItErrorCode::FAILURE;

  // initialization before start drawing
  while (ros::ok() && !init){
    ros::Duration(5).sleep(); // wait for 2 sec
    ROS_INFO("Sleeping 5 seconds before starting ... ");

    // move to init pose
    // set all the joint values to the init joint position
    move_group.setStartStateToCurrentState();
    move_group.setJointValueTarget("iiwa_joint_1", 0.0);
    move_group.setJointValueTarget("iiwa_joint_2", 0.435332);
    move_group.setJointValueTarget("iiwa_joint_3", 0.0);
    move_group.setJointValueTarget("iiwa_joint_4", -1.91986);
    move_group.setJointValueTarget("iiwa_joint_5", 0.0);
    move_group.setJointValueTarget("iiwa_joint_6", -0.785399);
    move_group.setJointValueTarget("iiwa_joint_7", 0.0);
    success_plan = move_group.plan(my_plan);
    if (success_plan == MoveItErrorCode::SUCCESS) {
      motion_done = move_group.execute(my_plan);
    }
    ROS_INFO("Moved to the initial position");
    ros::Duration(3).sleep(); // wait for 3 sec

    current_cartesian_position = move_group.getCurrentPose(ee_link);

    // save init position
    init_cartesian_position = command_cartesian_position = current_cartesian_position;
    // save end-effector orientation
    drawing_point = current_cartesian_position.pose;
    drawing_point.position.x += 0.03;   // 3cm depper
    drawing_point.position.z += 0.05;  // move up

    init = true;
  }

  bool ready_to_draw = false;

  // Drawing inputs
  DrawingInput drawing_k("/data/input/","quat1_",'k',".txt", drawing_point);

  int range_num = drawing_k.strokes_by_range.size();
  int j = 0;

  while(ros::ok() && init) {
    char input;
    for (int i = range_num-1; i >= 0; i--) {
      ///////////////////////////////////////////////////////////////////////
      // BLACK
      j = 0;
      for (auto strokes : drawing_k.strokes_by_range[i]) {
        command_cartesian_position.pose = strokes[0];
        command_cartesian_position.pose.position.x -= BACKWARD;

        // move to ready position
        linear_path.push_back(command_cartesian_position.pose);
        fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory);
        my_plan.trajectory_ = trajectory;
        move_group.execute(my_plan);  //ros::Duration(0.1).sleep();
        if (fraction < 0.5) ROS_WARN_STREAM("MOVE READY POSITION ERROR");
        linear_path.clear();

        // move forward
        command_cartesian_position.pose = strokes[0];
        linear_path.push_back(command_cartesian_position.pose);
        fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory);
        my_plan.trajectory_ = trajectory;
        move_group.execute(my_plan);  //ros::Duration(0.1).sleep();
        if (fraction < 0.5) ROS_WARN_STREAM("MOVING FORWARD ERROR");
        linear_path.clear();

        // draw
        cout << "Drawing BLACK " << i << "th range, " << j << "th stroke ... " << endl;
        fraction = move_group.computeCartesianPath(strokes, eef_step, jump_threshold, trajectory);
        my_plan.trajectory_ = trajectory;
        ros::Duration(0.1).sleep();
        motion_done = move_group.execute(my_plan);
        // if (motion_done == MoveItErrorCode::SUCCESS){
        //   visual_tools.publishTrajectoryLine(my_plan.trajectory_, link_model, joint_model_group, rviz_visual_tools::colors::WHITE);                   
        //   visual_tools.trigger();
        // }
        // else {
        //   ROS_WARN_STREAM("LINE EXECUTION ERROR");
        //   visual_tools.publishTrajectoryLine(my_plan.trajectory_, link_model, joint_model_group, rviz_visual_tools::colors::RED);
        //   visual_tools.trigger();
        // }
        ros::Duration(0.1).sleep();

        // move backward
        command_cartesian_position.pose = strokes.back();
        command_cartesian_position.pose.position.x -= BACKWARD;
        linear_path.push_back(command_cartesian_position.pose);
        fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory); // loosen the eef_step as moving backward does not need precision
        my_plan.trajectory_ = trajectory;

        move_group.execute(my_plan);  //ros::Duration(0.1).sleep();
        if (fraction < 0.5) ROS_WARN_STREAM("MOVE BACKWARD ERROR");
        linear_path.clear();
        j++;
      }

      move_group.setStartStateToCurrentState();
      move_group.setJointValueTarget("iiwa_joint_1", 0.0);
      move_group.setJointValueTarget("iiwa_joint_2", 0.435332);
      move_group.setJointValueTarget("iiwa_joint_3", 0.0);
      move_group.setJointValueTarget("iiwa_joint_4", -1.91986);
      move_group.setJointValueTarget("iiwa_joint_5", 0.0);
      move_group.setJointValueTarget("iiwa_joint_6", -0.785399);
      move_group.setJointValueTarget("iiwa_joint_7", 0.0);
      success_plan = move_group.plan(my_plan);
      if (success_plan == MoveItErrorCode::SUCCESS) {
        motion_done = move_group.execute(my_plan);
      }

      ROS_INFO("Moved to the initial position");
      ros::Duration(3).sleep(); // wait for 3 sec

      if (fraction < 0.5) ROS_WARN_STREAM("MOVE INIT ERROR");

      ///////////////////////////////////////////////////////////////////////

      // range done
      // move ridgeback
      if (i != 0) {
        float diff = (drawing_k.ranges[i][1] + drawing_k.ranges[i][0])/2 - (drawing_k.ranges[i-1][1] + drawing_k.ranges[i-1][0])/2;
        cout << "MOVE ridgeback " << diff << " meters in right and press any character with ENTER" << endl;
        std::cin >> input;
      }
    }

    init = false;
  }

  // All done
  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget("iiwa_joint_1", 0.0);
  move_group.setJointValueTarget("iiwa_joint_2", 0.435332);
  move_group.setJointValueTarget("iiwa_joint_3", 0.0);
  move_group.setJointValueTarget("iiwa_joint_4", -1.91986);
  move_group.setJointValueTarget("iiwa_joint_5", 0.0);
  move_group.setJointValueTarget("iiwa_joint_6", -0.785399);
  move_group.setJointValueTarget("iiwa_joint_7", 0.0);
  success_plan = move_group.plan(my_plan);
  if (success_plan == MoveItErrorCode::SUCCESS) {
    motion_done = move_group.execute(my_plan);
  }

  ROS_INFO("Moved to the initial position");
  ros::Duration(3).sleep(); // wait for 3 sec

  if (fraction < 0.5) ROS_WARN_STREAM("MOVE INIT ERROR");

  cerr<<"Stopping spinner..."<<endl;
  spinner.stop();

  cerr<<"Bye!"<<endl;

  return 0;
}
