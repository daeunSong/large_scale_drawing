#include "drawing_moveit.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include "boost/shared_ptr.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define PIXTOMM 0.26458
#define DRAWING_PATH "/input/ewha/"
#define DRAWING_FILENAME "ewha_full_path_"
#define RANGE 35

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
  // ros::Subscriber ir_sub = nh.subscribe("/iiwa_ridgeback_communicaiton/ridgeback", 100, chatterCallback);

  //*********** Read drawing file
  // std::ifstream txt(ros::package::getPath("large_scale_drawing")+DRAWING_PATH+DRAWING_FILENAME+"c.txt");
  // // check if text file is well opened
  // if(!txt.is_open()){
  //     std::cout << "FILE NOT FOUND\n";
  // }

  // divide drawing coordinates each RANGE
  // get width in pixel and convert it to mm
  // divide the width (mm) by range (to know how many times ridgeback needs to move)

  //*********** Init iiwa
  std::string movegroup_name, ee_link, planner_id, reference_frame;
  // dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh.param<std::string>("move_group", movegroup_name, PLANNING_GROUP);
  nh.param<std::string>("ee_link", ee_link, EE_LINK);
  nh.param<std::string>("planner_id", planner_id, PLANNER_ID);
  nh.param<std::string>("reference_frame", reference_frame, REFERENCE_FRAME);
  // set movegroup parameters and move iiwa to init pose
  DrawingMoveit iiwa(nh, movegroup_name, planner_id, ee_link, reference_frame);

  // //*********** Divide drawing coordinates by range (drawing inputs)
  geometry_msgs::Pose drawing_point;
  drawing_point = iiwa.getCurrentPose().pose;

  // DrawingInput drawing_c(DRAWING_PATH,DRAWING_FILENAME,'c',".txt", drawing_point);
  // DrawingInput drawing_m(DRAWING_PATH,DRAWING_FILENAME,'m',".txt", drawing_point);
  // DrawingInput drawing_y(DRAWING_PATH,DRAWING_FILENAME,'y',".txt", drawing_point);
  DrawingInput drawing_k(DRAWING_PATH,DRAWING_FILENAME,'k',".txt", drawing_point);

  //*********** Drawing and moving
  int range_num = drawing_k.strokes_by_range.size();
  for(int i = range_num-1; i >= 0; i--){
    // c, m, y, k
    iiwa.drawStrokes(nh, drawing_k, 'm', i); // iiwa draws
    
    std_msgs::String msg;
    msg.data = "1";
    ir_pub.publish(msg);  // ridgeback moves
    std::cout << "\n\n\n\n IIWA DONE \n\n";

    boost::shared_ptr<std_msgs::String const> ridgeback_done;
    ridgeback_done = ros::topic::waitForMessage<std_msgs::String>("/iiwa_ridgeback_communicaiton/ridgeback",nh);
    
    std::cout << "\n\n\n\n RIDGEBACK MOVED \n\n";
  }
}