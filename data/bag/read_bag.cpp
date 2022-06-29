#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/iiwa_ros.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Point.h>

#include <vector>
#include <string>
#include <iostream>

std::vector<std::string> split(const std::string input, const char delimiter){
  std::vector<std::string> dat;
  std::stringstream str(input);
  std::string temp;
  while(getline(str, temp, delimiter)){
      dat.push_back(temp);
  }
  return dat;
}

int main(int argc, char **argv){

  rosbag::Bag bag;
  bag.open("drawing.bag");

  std::string line;
  std::ifstream txt(ros::package::getPath("large_scale_drawing") + "/data/bag/time_stamp.txt");
  // check if text file is well opened
  if(!txt.is_open()){
    ROS_ERROR("FILE NOT FOUND");
    return 0;
  }
  // read time stamps first
  std::vector<std::tuple<double, double>> time_stamps;
  std::tuple<double, double> time_stamp;
  while(std::getline(txt, line)) {
    std::vector<std::string> tempSplit = split(line, ' ');
    time_stamp = {stod(tempSplit[1]), stod(tempSplit[2])};
    time_stamps.push_back(time_stamp);
  }

  std::ofstream outfile(ros::package::getPath("large_scale_drawing") + "/data/bag/drawing_traj.txt");
  geometry_msgs::Point drawing_point;

  int index = 0;
  double time;
  bool save = false;
  int tick = 0;

  // save trajectory from the bag file
  for (rosbag::MessageInstance const m: rosbag::View(bag))
  {
    iiwa_msgs::CartesianPose::ConstPtr i = m.instantiate<iiwa_msgs::CartesianPose>();
    if (i != nullptr)
    {
      drawing_point = i->poseStamped.pose.position;
      time = i->poseStamped.header.stamp.toSec();

      if (save)
      {
        if (time == std::get<1>(time_stamps[index]))  // end
        {
          save = false;
          outfile << "End\n";
          index ++;
          std::cout << i->poseStamped.header.stamp << std::endl;
          tick = 0;
        }
        else {
          if (tick > 21)
            outfile << std::to_string(drawing_point.x) << " " << std::to_string(drawing_point.y) << " " << std::to_string(drawing_point.z) << "\n";
          tick ++;
        }
      }
      else {
        if (time == std::get<0>(time_stamps[index]))  // start
        {
          save = true;
          std::cout << i->poseStamped.header.stamp << " ";
        }

      }

    }
  }
  outfile.close();
  std::cout << index << std::endl;

  return 0;
}
