#include "drawing_moveit.h"

#define BACKWARD 0.06

using moveit::planning_interface::MoveItErrorCode;

// Create MoveGroup
static const std::string PLANNING_GROUP = "manipulator";
static const std::string EE_LINK = "iiwa_link_ee";
/* RRTConnectkConfigDefault, RRTkConfigDefault, RRTstartkConfigDefault, TRRTkConfigDefault, ESTkConfigDefault
   SBLkConfigDefault, LBKPIECEkConfigDefault, BKPIECEkConfigDefault, PRMkConfigDefault, PRMstarkConfigDefault */
static const std::string PLANNER_ID = "RRTConnectkConfigDefault";
static const std::string REFERENCE_FRAME = "base_link";

bool sim;

DrawingMoveit::DrawingMoveit(ros::NodeHandle &nh, std::string planning_group, std::string planner_id, std::string ee_link_, std::string reference_frame){
  // to draw lines in rviz
  this->drawing_line = nh.advertise<std_msgs::Bool>("/ready_to_draw", 1);
  this->drawing_color = nh.advertise<geometry_msgs::Point>("/darwing_color", 1);
  
  // Create Move Group
  this->move_group = new moveit::planning_interface::MoveGroupInterface(planning_group);

  this->ee_link = ee_link_;
  // Configure Move Group
  this->move_group->setPlanningTime(0.5);
  this->move_group->setPlannerId(planner_id);
  this->move_group->setEndEffectorLink(ee_link);
  this->move_group->setPoseReferenceFrame(reference_frame);

  this->moveInitPose();
}

void DrawingMoveit::moveInitPose(){
  // move to init pose
  MoveItErrorCode success_plan = MoveItErrorCode::FAILURE;

  // set all the joint values to the init joint position
  this->move_group->setStartStateToCurrentState();
  this->move_group->setJointValueTarget("iiwa_joint_1", 0.0);
  this->move_group->setJointValueTarget("iiwa_joint_2", 0.435332);
  this->move_group->setJointValueTarget("iiwa_joint_3", 0.0);
  this->move_group->setJointValueTarget("iiwa_joint_4", -1.91986);
  this->move_group->setJointValueTarget("iiwa_joint_5", 0.0);
  this->move_group->setJointValueTarget("iiwa_joint_6", -0.785399);
  this->move_group->setJointValueTarget("iiwa_joint_7", 0.0);
  success_plan = this->move_group->plan(this->my_plan);
  if (success_plan == MoveItErrorCode::SUCCESS) {
    this->move_group->execute(this->my_plan);
  }
  ROS_INFO("Moved to the initial position");
  ros::Duration(3).sleep(); // wait for 3 sec
}

geometry_msgs::PoseStamped DrawingMoveit::getCurrentPose(){
  return this->move_group->getCurrentPose(this->ee_link);
}

void DrawingMoveit::savePose(){
  this->command_cartesian_position = this->getCurrentPose();
}

void DrawingMoveit::drawStrokes(ros::NodeHandle &nh, DrawingInput drawing_coor, std::string color_, int range_num){
  if(color_ == "cyan"){
    this->color.x = 0.0; this->color.y = 1.0; this->color.z = 1.0;   // cyan (0, 255, 255)
  }else if(color_ == "magenta"){
    this->color.x = 1.0; this->color.y = 0.0; this->color.z = 1.0;   // magenta (255, 0, 255)
  }else if(color_ == "yellow"){
    this->color.x = 1.0; this->color.y = 1.0; this->color.z = 0.0;   // yellow (255, 255, 0)
  }else if(color_ == "black"){
    this->color.x = 0.0; this->color.y = 0.0; this->color.z = 0.0;   // black (0, 0, 0)
  }

  std_msgs::Bool ready;
  
  int j = 0;
  double fraction = 0.0;
  this->drawing_color.publish(this->color);
  for (auto strokes : drawing_coor.strokes_by_range[range_num]) {
    this->command_cartesian_position.pose = strokes[0];
    this->command_cartesian_position.pose.position.x -= this->backward;

    // move to ready position
    this->linear_path.push_back(this->command_cartesian_position.pose);
    fraction = this->move_group->computeCartesianPath(this->linear_path, this->eef_step, this->jump_threshold, this->trajectory);
    this->my_plan.trajectory_ = this->trajectory;
    this->move_group->execute(this->my_plan);  //ros::Duration(0.1).sleep();
    if (fraction < 0.5) ROS_WARN_STREAM("MOVE READY POSITION ERROR");
    this->linear_path.clear();

    // move forward
    this->command_cartesian_position.pose = strokes[0];
    this->linear_path.push_back(this->command_cartesian_position.pose);
    fraction = this->move_group->computeCartesianPath(this->linear_path, this->eef_step, this->jump_threshold, this->trajectory);
    this->my_plan.trajectory_ = this->trajectory;
    this->move_group->execute(this->my_plan);  //ros::Duration(0.1).sleep();
    if (fraction < 0.5) ROS_WARN_STREAM("MOVING FORWARD ERROR");
    this->linear_path.clear();

    // draw
    std::cout << "Drawing "<<color_<<" " << range_num << "th range, " << j << "th stroke ... " << std::endl;
    fraction = this->move_group->computeCartesianPath(strokes, this->eef_step, this->jump_threshold, this->trajectory);
    this->my_plan.trajectory_ = this->trajectory;
    ros::Duration(0.1).sleep();
    ready.data = true;
    this->drawing_line.publish(ready);
    this->move_group->execute(this->my_plan);
    ros::Duration(0.1).sleep();
    ready.data = false;
    this->drawing_line.publish(ready);

    // move backward
    this->command_cartesian_position.pose = strokes.back();
    this->command_cartesian_position.pose.position.x -= this->backward;
    this->linear_path.push_back(this->command_cartesian_position.pose);
    fraction = this->move_group->computeCartesianPath(this->linear_path, this->eef_step, this->jump_threshold, this->trajectory); // loosen the eef_step as moving backward does not need precision
    this->my_plan.trajectory_ = this->trajectory;

    this->move_group->execute(this->my_plan);
    if (fraction < 0.5) ROS_WARN_STREAM("MOVE BACKWARD ERROR");
    this->linear_path.clear();
    
    j++;
  }

  this->moveInitPose();
}
