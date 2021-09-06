#include "iiwa_ros/iiwa_ros.hpp"
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/service/control_mode.hpp>
#include <iiwa_ros/conversions.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/package.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define TXT_FILE "/input/heart_path_y.txt"
#define BACKWARD 0.05
#define TRANSLATE_UP 0.43
#define TARGET_SIZE 0.5

using namespace std;
using moveit::planning_interface::MoveItErrorCode;

// Create MoveGroup
static const std::string PLANNING_GROUP = "manipulator";
static const std::string EE_LINK = "iiwa_link_ee";
/* RRTConnectkConfigDefault, RRTkConfigDefault, RRTstartkConfigDefault, TRRTkConfigDefault, ESTkConfigDefault
   SBLkConfigDefault, LBKPIECEkConfigDefault, BKPIECEkConfigDefault, PRMkConfigDefault, PRMstarkConfigDefault */
static const std::string PLANNER_ID = "RRTConnectkConfigDefault";
static const std::string REFERENCE_FRAME = "arm_mount_link";

bool sim;

vector<string> split(string input, char delimiter){
  vector<string> ans;
  stringstream str(input);
  string temp;

  while(getline(str, temp, delimiter)){
      ans.push_back(temp);
  }

  return ans;
}

vector<double> calculateNormal(vector<geometry_msgs::Pose> P){
  vector<double> normal;
  vector<double> P1, P2;

  // get 2 vectors from 3 points
  P1.push_back(P[0].position.x - P[1].position.x);
  P1.push_back(P[0].position.y - P[1].position.y);
  P1.push_back(P[0].position.z - P[1].position.z);
  P2.push_back(P[0].position.x - P[2].position.x);
  P2.push_back(P[0].position.y - P[2].position.y);
  P2.push_back(P[0].position.z - P[2].position.z);

  cout << "P1: " << P1[0] << " " << P1[1] << " " << P1[2] << endl;
  cout << "P2: " << P2[0] << " " << P2[1] << " " << P2[2] << endl << endl;

  // calculate normal vector
  normal.push_back(P1[1]*P2[2] - P1[2]*P2[1]);
  normal.push_back(P1[2]*P2[0] - P1[0]*P2[2]);
  normal.push_back(P1[0]*P2[1] - P1[1]*P2[0]);

  // make x of normal vector as 1
  normal[1] = normal[1]/normal[0];
  normal[2] = normal[2]/normal[0];
  normal[0] = 1;

  cout << "DIFF: " << P[0].position.x << " " << P[0].position.y << " " << P[0].position.z << endl;

  // find d (x + y + z + d = 0)
  double d = -1*P[0].position.x - normal[1]*P[0].position.y - normal[2]*P[0].position.z;
  normal.push_back(d);

  cout << endl << "NORMAL: " << normal[0] << " " << normal[1] << " " << normal[2] << " " << normal[3] << " " << endl << endl;

  return normal;
}

int main (int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "CommandRobotMoveit");
  ros::NodeHandle nh("~");
  nh.param("sim", sim, true);

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // iiwa_ros::iiwa_ros my_iiwa;
  // my_iiwa.init();
  // for Cartesian Impedance Control
  iiwa_ros::state::CartesianPose iiwa_pose_state;
  iiwa_ros::service::ControlModeService iiwa_control_mode;
  // Low stiffness only along Z.
  iiwa_msgs::CartesianQuantity cartesian_stiffness = iiwa_ros::conversions::CartesianQuantityFromFloat(1500,1500,350,300,300,300);
  iiwa_msgs::CartesianQuantity cartesian_damping = iiwa_ros::conversions::CartesianQuantityFromFloat(0.7);

  if(!sim){
    iiwa_pose_state.init("iiwa");
    iiwa_control_mode.init("iiwa");
  }

  std::string movegroup_name, ee_link, planner_id, reference_frame;
  geometry_msgs::PoseStamped current_cartesian_position, command_cartesian_position, start, end, init_cartesian_position;
  std::string joint_position_topic, cartesian_position_topic;
  std::vector<geometry_msgs::Pose> drawing_stroke;
  std::vector<geometry_msgs::Pose> linear_path;
  geometry_msgs::Pose drawing_point;
  geometry_msgs::Pose path_point;

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
  moveit_visual_tools::MoveItVisualTools visual_tools("odom");
  if (sim == true) {
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
  }

  // TXT file with list of coordinates
  ifstream txt(ros::package::getPath("large_scale_drawing")+TXT_FILE);
  // check if text file is well opened
  if(!txt.is_open()){
    cout << "FILE NOT FOUND" << endl;
    return 1;
  }

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


    ROS_INFO("The robot will be now set in Cartesian Impedance Mode");
    iiwa_control_mode.setCartesianImpedanceMode(cartesian_stiffness, cartesian_damping);
    current_cartesian_position = move_group.getCurrentPose(ee_link);

    init_cartesian_position = command_cartesian_position = current_cartesian_position; // save init position
    drawing_point = current_cartesian_position.pose;      // save end-effector orientation

    // FINDING NORMAL VECTOR TO TRANLSATE INPUT DATA
    // find distance between wall and set as x-value
    vector<geometry_msgs::Pose> P;
    int direction = -1;

    x = current_cartesian_position.pose.position.x + 0.03;

    /*
    for(int i = 0; i < 3; i++){
      current_cartesian_position = move_group.getCurrentPose(ee_link);
      current_cartesian_position.pose.position.x += BACKWARD;
      linear_path.push_back(current_cartesian_position.pose);
      fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory);
      my_plan.trajectory_ = trajectory;
      move_group.execute(my_plan);  //ros::Duration(0.1).sleep();
      if (fraction < 0.5) ROS_WARN_STREAM("MOVE FORWARD ERROR");

      linear_path.clear();

      ROS_INFO("Detecting the wall");
      ROS_INFO("Sleeping 3 seconds before starting ... ");
      ros::Duration(3).sleep(); // wait for 3 sec

      // save the wall's x position
      current_cartesian_position = move_group.getCurrentPose(ee_link);
      P.push_back(current_cartesian_position.pose);  // default x position

      // move backward
      current_cartesian_position.pose.position.x -= 0.01;
      linear_path.push_back(current_cartesian_position.pose);
      fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory);
      my_plan.trajectory_ = trajectory;
      move_group.execute(my_plan);  //ros::Duration(0.1).sleep();
      if (fraction < 0.5) ROS_WARN_STREAM("MOVE BACKWARD ERROR");

      linear_path.clear();
      ros::Duration(2).sleep(); // wait for 2 sec

      // move diagonal
      current_cartesian_position = init_cartesian_position;
      current_cartesian_position.pose.position.y -= 0.05;
      current_cartesian_position.pose.position.z += direction*0.02;
      linear_path.push_back(current_cartesian_position.pose);
      fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory);
      my_plan.trajectory_ = trajectory;
      move_group.execute(my_plan);  //ros::Duration(0.1).sleep();
      if (fraction < 0.5) ROS_WARN_STREAM("MOVE BACKWARD ERROR");

      direction *= -1;
    }
    */

//    //sample for simulation
//    current_cartesian_position.pose.position.x += 0.02;
//    P.push_back(current_cartesian_position.pose);
//    current_cartesian_position.pose.position.z += 0.04;
//    P.push_back(current_cartesian_position.pose);
//    current_cartesian_position.pose.position.x += 0.007;
//    current_cartesian_position.pose.position.y += 0.04;
//    current_cartesian_position.pose.position.z -= 0.04;
//    P.push_back(current_cartesian_position.pose);
//
//    cout << "IIWA_POSITION: " << init_cartesian_position.pose.position.x << " " << init_cartesian_position.pose.position.y << " " << init_cartesian_position.pose.position.z << endl;
//
//    normal = calculateNormal(P);

    init = true;
  }

//  cout << "TESTING: " << sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]+normal[2]) << " " << normal[0] / sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]+normal[2]) << endl;
//  double ang = acos(normal[0] / sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]+normal[2]));
//  //if(sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]+normal[2]) < 1) ang = 0;
//  double y_formula;  // save b from y = az + b as a is 0
//                     // a = -1*normal[2]/normal[1];
//  y_formula = -1*(normal[3])/normal[1];
//
//  cout << endl << "Y_FORMULA: " <<  y_formula << "ang: " << ang << endl << endl;

  int stroke_num = 0;
  bool ready_to_draw = false;

  getline(txt, line); // drawing size
  vector<string> tempSplit_ = split(line, ' ');
  double width = stod(tempSplit_[0]);
  double height = stod(tempSplit_[1]);
  double ratio = width / height;

  while(ros::ok() && getline(txt, line) && init){
    if(line == "End"){
      stroke_num++;

      ROS_INFO("The robot will be now set in Cartesian Impedance Mode");
      iiwa_control_mode.setCartesianImpedanceMode(cartesian_stiffness, cartesian_damping);

      // move forward first to draw
      ROS_INFO("Moving Forward ... ");
      command_cartesian_position.pose = drawing_stroke[0];
      linear_path.push_back(command_cartesian_position.pose);
      fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory);
      my_plan.trajectory_ = trajectory;
      move_group.execute(my_plan);  //ros::Duration(0.1).sleep();
      if (fraction < 0.5) ROS_WARN_STREAM("MOVE FORWARD ERROR");

      linear_path.clear();

      // draw a stroke
      ROS_INFO("Drawing %d th stroke ...", stroke_num);
      fraction = move_group.computeCartesianPath(drawing_stroke, eef_step, jump_threshold, trajectory);
      my_plan.trajectory_ = trajectory;

      ROS_INFO("Visualizing drawing plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
      if (fraction < 0.5) ROS_WARN_STREAM("LINE DRAWING ERROR");

      motion_done = move_group.execute(my_plan); //ros::Duration(0.1).sleep();
      if (sim == true){   // Rviz drawing visualization
        if (motion_done == MoveItErrorCode::SUCCESS){
          visual_tools.publishTrajectoryLine(my_plan.trajectory_, link_model, joint_model_group, rviz_visual_tools::colors::WHITE);
          visual_tools.trigger();
        }
        else {
          ROS_WARN_STREAM("LINE EXECUTION ERROR");
          visual_tools.publishTrajectoryLine(my_plan.trajectory_, link_model, joint_model_group, rviz_visual_tools::colors::RED);
          visual_tools.trigger();
        }
      }

      // getCurrentPose ignores the reference frame, thus get the latest position from drawing_stroke
      ROS_INFO("Moving Backward ... \n");
      command_cartesian_position.pose = drawing_stroke.back();

      // move backward
      // linear_path.push_back(command_cartesian_position.pose);
      command_cartesian_position.pose.position.x -= BACKWARD;
      linear_path.push_back(command_cartesian_position.pose);
      fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory); // loosen the eef_step as moving backward does not need precision
      my_plan.trajectory_ = trajectory;

      move_group.execute(my_plan);  //ros::Duration(0.1).sleep();
      if (fraction < 0.5) ROS_WARN_STREAM("MOVE BACKWARD ERROR");

      linear_path.clear();
      // linear_path.push_back(command_cartesian_position.pose);

      ready_to_draw = false;
      drawing_stroke.clear();
    }
    else{
      // read drawing
      vector<string> tempSplit = split(line, ' ');
      y = (stod(tempSplit[0])-0.5) * ratio * TARGET_SIZE;
      z = (-stod(tempSplit[1])+0.5) * TARGET_SIZE + TRANSLATE_UP;

//      if(ang != 0){ // if ridgeback is not parallel to the wall
//        double radius = abs(y_formula - y);
//        // cout << "CALCULATING: " << ang << " " << sin(ang) << " " << cos(ang) << " " << radius << endl;
//        x = radius * sin(ang); //+ init_cartesian_position.pose.position.x;
//        y = radius * cos(ang) + y_formula;
//      }

      /* make scale smaller
      y = 0.8*y;
      z = 0.8*z; */

      if (!ready_to_draw){
        // move to the ready position (off the wall)
        ROS_INFO("The robot will be now set in Position Control Mode");
        iiwa_control_mode.setPositionControlMode();

        ROS_INFO("Moving To Ready Position ... ");
        command_cartesian_position.pose.position.x = x - BACKWARD;
        command_cartesian_position.pose.position.y = y;
        command_cartesian_position.pose.position.z = z;

        linear_path.push_back(command_cartesian_position.pose);
        fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory);
        my_plan.trajectory_ = trajectory;
        move_group.execute(my_plan);  //ros::Duration(0.1).sleep();
        if (fraction < 0.5) ROS_WARN_STREAM("MOVE READY POSITION ERROR");

        linear_path.clear();
        // linear_path.push_back(command_cartesian_position.pose);
        ready_to_draw = true;
      }

      drawing_point.position.x = x;
      drawing_point.position.y = y;
      drawing_point.position.z = z;

      drawing_stroke.push_back(drawing_point); // push the point
    }
  }

  linear_path.clear();

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
