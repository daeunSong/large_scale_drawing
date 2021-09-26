#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>


bool ready_to_draw = false;
geometry_msgs::Point color;

// Callback function to know whether if ready to visualize drawing
void drawCallback(const std_msgs::Bool::ConstPtr& msg){
  ready_to_draw = msg->data;
}

// Callback function to get the drawing color
void colorCallback(const geometry_msgs::Point::ConstPtr& msg){
  color = *msg;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 100);
  ros::Subscriber drawing_sub = nh.subscribe("/ready_to_draw", 10, drawCallback);
  ros::Subscriber color_sub = nh.subscribe("/darwing_color", 10, colorCallback);
  tf::TransformListener listener;
  tf::StampedTransform transform;

//  ros::AsyncSpinner spinner(1);
//  spinner.start();

  geometry_msgs::Point p;

  float id = 0.0;

  visualization_msgs::Marker line_strip;

  line_strip.header.frame_id = "/base_link";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  line_strip.pose.orientation.w = 1.0;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.001;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // Set the color -- be sure to set alpha to something non-zero!
    line_strip.color.a = 1.0;
    line_strip.color.r = color.x;
    line_strip.color.g = color.y;
    line_strip.color.b = color.z;

    try {
      listener.waitForTransform("/base_link", "/iiwa_link_ee", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("/base_link", "/iiwa_link_ee", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
     }

    // marker init
    line_strip.header.stamp = ros::Time::now();
    line_strip.id = id;

//    while(sub_msg.x > 0.55){
    p.x = transform.getOrigin().x();
    p.y = transform.getOrigin().y();
    p.z = transform.getOrigin().z();

    if (ready_to_draw) {
      line_strip.points.push_back(p);

       if(line_strip.points.size() > 10){
         marker_pub.publish(line_strip);
         line_strip.points.erase(line_strip.points.begin());
         line_strip.points.push_back(p);
       }
    }

    id++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
