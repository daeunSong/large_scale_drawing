#include "line_publisher.h"

LinePublisher::LinePublisher() : nh("") {
  initSubscriber();
  initPublisher();
  initMarker();
}

// Init subscriber
void LinePublisher::initSubscriber() {
  drawing_sub = nh.subscribe("/ready_to_draw", 10, &LinePublisher::drawCallback, this);
  color_sub = nh.subscribe("/darwing_color", 10, &LinePublisher::colorCallback, this);
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
  line_strip.color.a = 1.0;
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
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "points_and_lines");

  LinePublisher linePublisher;
  ros::Rate loop_rate(10);
  float id = 0.0;

  while (ros::ok()) {
    linePublisher.publishLine(id);

    id++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}