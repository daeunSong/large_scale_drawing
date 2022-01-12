#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

class MarkerPublisher {
  public:
    MarkerPublisher();

    bool ready_to_draw = false;
    visualization_msgs::Marker line_strip;
    visualization_msgs::Marker wall;

    ros::Publisher marker_pub;

    geometry_msgs::Point getEEPoint();
    void setColor();
    void publishLine(float id);
    void initWall(std::string wall_file_name, std::vector<double> wall_position);

  private:
    ros::NodeHandle nh;

    geometry_msgs::Point line_color;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    void initSubscriber();
    void initPublisher();
    void initMarker();

    ros::Subscriber drawing_sub;
    ros::Subscriber color_sub;

    void drawCallback(const std_msgs::Bool::ConstPtr& msg);
    void colorCallback(const geometry_msgs::Point::ConstPtr& msg);
};
