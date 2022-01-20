#include "marker_publisher.h"

MarkerPublisher::MarkerPublisher() : nh("") {
  // init visual tools
  this->visual_tools_.reset(new rvt::RvizVisualTools("/odom", "/axes_marker"));
  this->visual_tools_->loadMarkerPub();
  // Clear messages
  this->visual_tools_->deleteAllMarkers();
  this->visual_tools_->enableBatchPublishing();

  initSubscriber();
  initPublisher();
  initMarker();
}

// Init subscriber
void MarkerPublisher::initSubscriber() {
  drawing_sub = nh.subscribe("/ready_to_draw", 10, &MarkerPublisher::drawCallback, this);
  color_sub = nh.subscribe("/drawing_color", 10, &MarkerPublisher::colorCallback, this);
  traj_sub = nh.subscribe("/iiwa_ridgeback_communicaiton/trajectory", 100, &MarkerPublisher::trajCallback, this);
//  coord_sub = nh.subscribe("/coord", 10, &MarkerPublisher::coordCallback, this); //
}

// Init publisher
void MarkerPublisher::initPublisher() {
  marker_pub = nh.advertise<visualization_msgs::Marker>("/drawing_marker", 100);
}

// Init marker
void MarkerPublisher::initMarker() {
  line_strip.header.frame_id = "/odom";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  line_strip.pose.orientation.w = 1.0;
  line_strip.scale.x = 0.001;
}

// init
void MarkerPublisher::initWall() {
  visualization_msgs::Marker wall_marker;
  wall_marker.header.frame_id = "/odom";
  wall_marker.header.stamp = ros::Time::now();
  wall_marker.ns = "wall";
  wall_marker.id = 0;
  wall_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  wall_marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Pose marker_pose;
  marker_pose.position.x = wall_pose[0] + 0.015;
  marker_pose.position.y = wall_pose[1] + 0.015;
  marker_pose.position.z = wall_pose[2];
  marker_pose.orientation.x = wall_pose[3];
  marker_pose.orientation.y = wall_pose[4];
  marker_pose.orientation.z = wall_pose[5];
  marker_pose.orientation.w = wall_pose[6];

  wall_marker.pose = marker_pose;
  wall_marker.scale.x = 1.0;
  wall_marker.scale.y = 1.0;
  wall_marker.scale.z = 1.0;

  wall_marker.color.a = 1.0;
  wall_marker.color.r = 0.933;
  wall_marker.color.g = 0.933;
  wall_marker.color.b = 0.933;

  //only if using a MESH_RESOURCE marker type:
  wall_marker.mesh_resource = "package://large_scale_drawing/data/wall/"+wall_file_name+".obj";
  marker_pub.publish(wall_marker);
}

// Callback function to know whether if ready to visualize drawing
void MarkerPublisher::drawCallback(const std_msgs::Bool::ConstPtr& msg){
  ready_to_draw = msg->data;
}

// Callback function to get the drawing color
void MarkerPublisher::colorCallback(const geometry_msgs::Point::ConstPtr& msg){
  line_color = *msg;
  setColor();
}

// Callback function to get the ridgeback trajectory
void MarkerPublisher::trajCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
  trajectories = *msg;

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  for (int i = 0; i < trajectories.poses.size(); i++) {
    pose = Eigen::Isometry3d::Identity();
    // translation
    pose.translation().x() = trajectories.poses[i].position.x + wall_pose[0];
    pose.translation().y() = trajectories.poses[i].position.y + wall_pose[1];
    // rotation (we have sent z-axis rotation angle in orientation.x, euler given radian needed)
    pose = pose * Eigen::AngleAxisd(trajectories.poses[i].orientation.x * D2R, Eigen::Vector3d::UnitZ());
    this->visual_tools_->publishAxis(pose);
  }
  this->visual_tools_->trigger();
}

// publish axis
void MarkerPublisher::coordCallback(const geometry_msgs::Pose::ConstPtr& msg){
  geometry_msgs::Pose marker_pose = *msg;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  // rotation
  Eigen::Quaterniond q(marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w);
  Eigen::Matrix3d R = q.toRotationMatrix();
  pose = R * pose;
  // translation
  pose.translation().x() = marker_pose.position.x;
  pose.translation().y() = marker_pose.position.y;
  pose.translation().z() = marker_pose.position.z;

  this->visual_tools_->publishAxis(pose);
  this->visual_tools_->trigger();
  std::cout << "AXIS PUBLISHED" << std::endl;
}

// Get the end-effector pose
geometry_msgs::Point MarkerPublisher::getEEPoint(){
  try {
    listener.waitForTransform("/odom", "/iiwa_link_ee", ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("/odom", "/iiwa_link_ee", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  geometry_msgs::Point p;
  p.x = transform.getOrigin().x();
  p.y = transform.getOrigin().y();
  p.z = transform.getOrigin().z();
  return p;
}

// Set color
void MarkerPublisher::setColor(){
  line_strip.color.a = 0.5;
  line_strip.color.r = line_color.x;
  line_strip.color.g = line_color.y;
  line_strip.color.b = line_color.z;
}

void MarkerPublisher::publishLine(float id) {
  geometry_msgs::Point p;

  p = getEEPoint();
  line_strip.header.stamp = ros::Time::now();
  line_strip.id = id;

  if (ready_to_draw) {
    line_strip.points.push_back(p);
    if(line_strip.points.size() > 10){
      marker_pub.publish(line_strip);
      line_strip.points.erase(line_strip.points.begin());
      line_strip.points.push_back(p);
    }
  }
  else {
    line_strip.points.clear();
  }
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "marker_publisher");
  ros::NodeHandle nh("~");

  MarkerPublisher markerPublisher;

  nh.getParam("/wall_pose", markerPublisher.wall_pose);
  nh.getParam("/wall_file_name", markerPublisher.wall_file_name);

  ros::Rate loop_rate(10);
  float id = 0.0;
  bool init = false;

  while (ros::ok()) {
    if (!init) {
      ros::Duration(10.0).sleep();
      markerPublisher.initWall();
      init = true;
      ros::Duration(1.0).sleep();
    }
    markerPublisher.publishLine(id);

    id++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
