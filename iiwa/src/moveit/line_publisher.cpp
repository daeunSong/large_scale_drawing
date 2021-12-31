#include "line_publisher.h"

LinePublisher::LinePublisher() : nh("") {
  initSubscriber();
  initPublisher();
  initMarker();
}

// Init subscriber
void LinePublisher::initSubscriber() {
  drawing_sub = nh.subscribe("/ready_to_draw", 10, &LinePublisher::drawCallback, this);
  color_sub = nh.subscribe("/drawing_color", 10, &LinePublisher::colorCallback, this);
}

// Init publisher
void LinePublisher::initPublisher() {
  marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 100);
}

// Init marker
void LinePublisher::initMarker() {
  line_strip.header.frame_id = "/odom";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  line_strip.pose.orientation.w = 1.0;
  line_strip.scale.x = 0.001;
}

void LinePublisher::initWall() {
  visualization_msgs::Marker wall_marker;
  wall_marker.header.frame_id = "/odom";
  wall_marker.header.stamp = ros::Time::now();
  wall_marker.ns = "wall";
  wall_marker.id = 0;
  wall_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  wall_marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Pose wall_pose;
  wall_pose.orientation.w = 1.0;
  wall_pose.position.x = 0.85;
  wall_pose.position.y = 0.645; // TODO //drawing_coor.wall_center + (drawing_coor.ranges[0][0]+drawing_coor.ranges[0][1])/2;
  // 0.869999 + -0.225
  wall_pose.position.z = 0.0;

  wall_marker.pose = wall_pose;
  wall_marker.scale.x = 1.0;
  wall_marker.scale.y = 1.0;
  wall_marker.scale.z = 1.0;

  wall_marker.color.a = 1.0; // Don't forget to set the alpha!
  wall_marker.color.r = 0.933;
  wall_marker.color.g = 0.933;
  wall_marker.color.b = 0.933;

  //only if using a MESH_RESOURCE marker type:
  wall_marker.mesh_resource = "package://large_scale_drawing/wall/bee_hive.obj";
  marker_pub.publish(wall_marker);
}


// Callback function to know whether if ready to visualize drawing
void LinePublisher::drawCallback(const std_msgs::Bool::ConstPtr& msg){
  ready_to_draw = msg->data;
}

// Callback function to get the drawing color
void LinePublisher::colorCallback(const geometry_msgs::Point::ConstPtr& msg){
  line_color = *msg;
  setColor();
}

// Get the end-effector pose
geometry_msgs::Point LinePublisher::getEEPoint(){
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
void LinePublisher::setColor(){
  line_strip.color.a = 0.5;
  line_strip.color.r = line_color.x;
  line_strip.color.g = line_color.y;
  line_strip.color.b = line_color.z;
}

void LinePublisher::publishLine(float id) {
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
  ros::init(argc, argv, "points_and_lines");

  LinePublisher linePublisher;
  ros::Rate loop_rate(10);
  float id = 0.0;
  bool init = true;

  while (ros::ok()) {
    if (init) {
      ros::Duration(5.0).sleep();
      linePublisher.initWall();
      init = false;
    }
    linePublisher.publishLine(id);

    id++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
