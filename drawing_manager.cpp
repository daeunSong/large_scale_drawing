#include "drawing_moveit.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include "boost/shared_ptr.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

//// drawing file
//#define DRAWING_PATH "/input/ewha/"
//#define DRAWING_FILENAME "ewha_full_path_"
//// wall file
//#define WALL_FILENAME "/wall/bee_hive.obj"
// drawing parameters
#define BACKWARD  0.06


// Create MoveGroup
static const std::string PLANNING_GROUP = "manipulator";
static const std::string EE_LINK = "iiwa_link_ee";
/* RRTConnectkConfigDefault, RRTkConfigDefault, RRTstartkConfigDefault, TRRTkConfigDefault, ESTkConfigDefault
   SBLkConfigDefault, LBKPIECEkConfigDefault, BKPIECEkConfigDefault, PRMkConfigDefault, PRMstarkConfigDefault */
static const std::string PLANNER_ID = "RRTConnectkConfigDefault";
static const std::string REFERENCE_FRAME = "base_link";

int main(int argc, char **argv){
  //*********** Initialize ROS
  ros::init(argc, argv, "CommandRobotMoveit");
  ros::NodeHandle nh("~");

  //*********** ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //*********** MSG publisher
  ros::Publisher ir_pub = nh.advertise<std_msgs::String>("/iiwa_ridgeback_communicaiton/iiwa", 1);
  std_msgs::String iiwa_state;


  //*********** Init IIWA
  std::string move_group_name, ee_link, planner_id, reference_frame;
  // dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh.param<std::string>("move_group", move_group_name, PLANNING_GROUP);
  nh.param<std::string>("ee_link", ee_link, EE_LINK);
  nh.param<std::string>("planner_id", planner_id, PLANNER_ID);
  nh.param<std::string>("reference_frame", reference_frame, REFERENCE_FRAME);
  // set movegroup parameters and move iiwa to init pose
  DrawingMoveit iiwa(nh, move_group_name, planner_id, ee_link, reference_frame);
  ros::Duration(1.0).sleep();

  geometry_msgs::Pose init_drawing_pose;
  init_drawing_pose = iiwa.getCurrentPose().pose;
  init_drawing_pose.position.x += 0.03;   // 3cm depper
  init_drawing_pose.position.z += 0.05;  // move up
  
  //*********** Init Ridgeback (get drawing split ranges)
  bool init = true;
  boost::shared_ptr<std_msgs::Float64MultiArray const> drawing_ranges;


  //*********** Init Problem
  std::string wall_file_name, drawing_file_name;
  std::vector<double> wall_pose;
  nh.getParam("/wall_file_name", wall_file_name);
  nh.getParam("/wall_pose", wall_pose);
  nh.getParam("/drawing_file_name", drawing_file_name);
  // init drawing
  DrawingInput drawing_k(wall_file_name, drawing_file_name, 'k', init_drawing_pose);
  // Set the wall


  ROS_INFO("Waiting for drawing ranges");
  while(ros::ok() && init){
    drawing_ranges = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/iiwa_ridgeback_communicaiton/drawing_range",nh);
    init = false;
  }
  std_msgs::Float64MultiArray ranges = *drawing_ranges;
  ROS_INFO("Got drawing_range from Ridgeback");

//  //*********** Read drawing file (drawing coordinates divided by range)
//  geometry_msgs::Pose drawing_point;
//  drawing_point = iiwa.getCurrentPose().pose;
//  drawing_point.position.x += 0.03;   // 3cm depper
//  drawing_point.position.z += 0.05;  // move up
//
//  iiwa_state.data = "1";
//  ir_pub.publish(iiwa_state);  // tell ridgeback to stop moving
//
//  // DrawingInput drawing_c(DRAWING_PATH,DRAWING_FILENAME,'c',".txt", init_drawing_pose);
//  // DrawingInput drawing_m(DRAWING_PATH,DRAWING_FILENAME,'m',".txt", init_drawing_pose);
//  // DrawingInput drawing_y(DRAWING_PATH,DRAWING_FILENAME,'y',".txt", init_drawing_pose);
//  DrawingInput drawing_k(WALL_FILENAME, ranges, DRAWING_PATH, DRAWING_FILENAME, 'k', ".txt", init_drawing_pose);
//
//  ROS_INFO("Drawing Inputs Ready!");
//
//  splitByRangeArb(ri_ranges)
//
//  //*********** Drawing and moving
//  int range_num = drawing_k.strokes_by_range.size();
//  init = true;
//
//  iiwa_state.data = "0";
//  ir_pub.publish(iiwa_state);  // tell ridgeback to stop moving
//
//  while(ros::ok() && init){
//    for(int i = range_num-1; i >= 0; i--){
//      // get ridgeback's position and orientation
//      ROS_INFO("Waiting for ridgeback's position and orientation");
//      boost::shared_ptr<geometry_msgs::Pose const> ri_poses;
//      ri_poses = ros::topic::waitForMessage<geometry_msgs::Pose>("/iiwa_ridgeback_communicaiton/ridgeback/pose",nh);
//      geometry_msgs::Pose poses = *ri_poses;
//
//      iiwa_state.data = "1";
//      ir_pub.publish(iiwa_state);  // tell ridgeback to stop moving
//
//      // relocate drawing coordinate according to ridgeback's location
//      ROS_INFO("Relocate drawing coordinate according to ridgeback's pose");
//      drawing_k.relocateDrawingsArb(poses, i);
//
//      // iiwa draw (color: c, m, y, k)
//      ROS_INFO("IIWA Drawing Start");
//      iiwa.drawStrokes(nh, drawing_k, 'm', i);
//
//      // finished iiwa drawing make ridgeback move
//      iiwa_state.data = "0";
//      ir_pub.publish(iiwa_state);  // ridgeback moves
//      std::cout << "\n\n\n\n IIWA DONE \n\n";
//
//      // wait for ridgeback to finish moving
//      ROS_INFO("Waiting for ridgeback to finish moving");
//      // boost::shared_ptr<std_msgs::String const> ridgeback_done;
//      // ridgeback_done = ros::topic::waitForMessage<std_msgs::String>("/iiwa_ridgeback_communicaiton/ridgeback/state",nh);
//
//      std::cout << "\n\n\n\n RIDGEBACK MOVED \n\n";
//      // iiwa_state.data = "1";
//      // ir_pub.publish(iiwa_state);  // tell ridgeback to stop moving
//    }
//
//    init = false;
//  }
}
