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
  this->drawing_color = nh.advertise<geometry_msgs::Point>("/drawing_color", 1);
  
  // Create Move Group
  this->move_group = new moveit::planning_interface::MoveGroupInterface(planning_group);

  this->ee_link = ee_link_;
  // Configure Move Group
  this->move_group->setPlanningTime(0.5);
  this->move_group->setPlannerId(planner_id);
  this->move_group->setEndEffectorLink(ee_link);
  this->move_group->setPoseReferenceFrame(reference_frame);

  // Initialize the planning scene
//  this->initScene();
  // Move robot to the initial pose
  this->moveInitPose();
}

void DrawingMoveit::addScene(int j){
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = this->move_group->getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "wall";

  // Define a mesh to add to the world.
  shapes::Mesh* mesh = shapes::createMeshFromResource("package://large_scale_drawing/wall/bee_hive.obj", {1.0, 1.0, 1.0}); // TODO
  shape_msgs::Mesh wall_mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(mesh, mesh_msg);
  wall_mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose wall_pose;
  wall_pose.orientation.w = 1.0;
  wall_pose.position.x = 0.83; // TODO
  wall_pose.position.y = 0.869999 - 0.225 + (0.45 * (j-1)); // 0.645; // TODO //drawing_coor.wall_center + (drawing_coor.ranges[0][0]+drawing_coor.ranges[0][1])/2;
  // 0.869999 + -0.225
  wall_pose.position.z = 0.0;

  collision_object.meshes.push_back(wall_mesh);
  collision_object.mesh_poses.push_back(wall_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Add the collision object into the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ros::Duration(3.0).sleep();
  ROS_INFO("Collision object added");
  planning_scene_interface.addCollisionObjects(collision_objects);
  ros::Duration(3.0).sleep();
}

void DrawingMoveit::moveInitPose(){
  // move to init pose
  MoveItErrorCode success_plan = MoveItErrorCode::FAILURE;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // set all the joint values to the init joint position
  this->move_group->setStartStateToCurrentState();
  this->move_group->setJointValueTarget("iiwa_joint_1", 0.0);
  this->move_group->setJointValueTarget("iiwa_joint_2", 0.435332);
  this->move_group->setJointValueTarget("iiwa_joint_3", 0.0);
  this->move_group->setJointValueTarget("iiwa_joint_4", -1.91986);
  this->move_group->setJointValueTarget("iiwa_joint_5", 0.0);
  this->move_group->setJointValueTarget("iiwa_joint_6", -0.785399);
  this->move_group->setJointValueTarget("iiwa_joint_7", 0.0);
  success_plan = this->move_group->plan(my_plan);
  if (success_plan == MoveItErrorCode::SUCCESS) {
    this->move_group->execute(my_plan);
  }
  ROS_INFO("Moved to the initial position");
  ros::Duration(3).sleep(); // wait for 3 sec
}

geometry_msgs::PoseStamped DrawingMoveit::getCurrentPose(){
  return this->move_group->getCurrentPose(this->ee_link);
}

void DrawingMoveit::drawStrokes(ros::NodeHandle &nh, DrawingInput &drawing_coor, char color_, int range_num){
  if(color_ == 'c'){
    this->color.x = 0.0; this->color.y = 1.0; this->color.z = 1.0;   // cyan (0, 255, 255)
  }else if(color_ == 'm'){
    this->color.x = 1.0; this->color.y = 0.0; this->color.z = 1.0;   // magenta (255, 0, 255)
  }else if(color_ == 'y'){
    this->color.x = 1.0; this->color.y = 1.0; this->color.z = 0.0;   // yellow (255, 255, 0)
  }else if(color_ == 'k'){
    this->color.x = 0.0; this->color.y = 0.0; this->color.z = 0.0;   // black (0, 0, 0)
  }

  this->addScene(drawing_coor.strokes_by_range.size() - range_num);

 // drawing commands related
  geometry_msgs::PoseStamped command_cartesian_position;
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> linear_path;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std_msgs::Bool ready;
  int j = 0;
  double fraction = 0.0;
  this->drawing_color.publish(this->color);
  for (auto strokes : drawing_coor.strokes_by_range[range_num]) {
    command_cartesian_position.pose = strokes[0];
    command_cartesian_position.pose.position.x -= this->backward;

    // move to ready position
    linear_path.push_back(command_cartesian_position.pose);
    fraction = this->move_group->computeCartesianPath(linear_path, this->eef_step, this->jump_threshold, trajectory);
    my_plan.trajectory_ = trajectory;
    this->move_group->execute(my_plan);  //ros::Duration(0.1).sleep();
    if (fraction < 0.5) ROS_WARN_STREAM("MOVE READY POSITION ERROR");
    linear_path.clear();

    // move forward
    command_cartesian_position.pose = strokes[0];
    linear_path.push_back(command_cartesian_position.pose);
    fraction = this->move_group->computeCartesianPath(linear_path, this->eef_step, this->jump_threshold, trajectory);
    my_plan.trajectory_ = trajectory;
    this->move_group->execute(my_plan);  //ros::Duration(0.1).sleep();
    if (fraction < 0.5) ROS_WARN_STREAM("MOVING FORWARD ERROR");
    linear_path.clear();

    // draw
    std::cout << "Drawing "<<color_<<" " << range_num << "th range, " << j << "th stroke ... " << std::endl;
    fraction = this->move_group->computeCartesianPath(strokes, this->eef_step, this->jump_threshold, trajectory);
    my_plan.trajectory_ = trajectory;
    ros::Duration(0.1).sleep();
    ready.data = true;
    this->drawing_line.publish(ready);

    this->move_group->execute(my_plan);
    ros::Duration(0.1).sleep();
    ready.data = false;
    this->drawing_line.publish(ready);

    // move backward
    command_cartesian_position.pose = strokes.back();
    command_cartesian_position.pose.position.x -= this->backward;
    linear_path.push_back(command_cartesian_position.pose);
    fraction = this->move_group->computeCartesianPath(linear_path, this->eef_step, this->jump_threshold, trajectory); // loosen the eef_step as moving backward does not need precision
    my_plan.trajectory_ = trajectory;

    this->move_group->execute(my_plan);
    if (fraction < 0.5) ROS_WARN_STREAM("MOVE BACKWARD ERROR");
    linear_path.clear();
    
    j++;
  }

  this->moveInitPose();
}
