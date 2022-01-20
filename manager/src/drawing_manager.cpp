#include "drawing_manager.h"

DrawingManager::DrawingManager(ros::NodeHandle* nh):nh_(*nh) {
  // init param
  nh_.getParam("/wall_file_name", wall_file_name);
  nh_.getParam("/wall_pose", wall_pose);
  nh_.getParam("/drawing_file_name", drawing_file_name);

  initSubscriber();
  initPublisher();
  initMarker();
  initMoveGroup();
}

// Init subscriber
void DrawingManager::initSubscriber() {
  ir_sub = nh_.subscribe("/iiwa_ridgeback_communicaiton/ridgeback/state", 10, &DrawingManager::stateCallback, this);
}

// Init publisher
void DrawingManager::initPublisher() {
  ir_pub = nh_.advertise<std_msgs::String>("/iiwa_ridgeback_communicaiton/iiwa/state", 1);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("/target_drawing", 100);
}

// Init marker for target drawing
void DrawingManager::initMarker() {
  marker.header.frame_id = "/odom";
  marker.header.stamp = ros::Time::now();
  marker.ns = "target";
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;

  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.001;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.3;

//  // TODO: remove
//  target_range_marker.header.frame_id = "/base_link";
//  target_range_marker.header.stamp = ros::Time::now();
//  target_range_marker.ns = "target_moved";
//  target_range_marker.action = visualization_msgs::Marker::ADD;
//  target_range_marker.type = visualization_msgs::Marker::LINE_STRIP;
//
//  target_range_marker.pose.orientation.w = 1.0;
//  target_range_marker.scale.x = 0.001;
//  target_range_marker.color.r = 1.0;
//  target_range_marker.color.g = 0.0;
//  target_range_marker.color.b = 0.0;
//  target_range_marker.color.a = 0.3;
}

void DrawingManager::initMoveGroup() {
  // dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh_.param<std::string>("move_group", move_group_name, PLANNING_GROUP);
  nh_.param<std::string>("ee_link", ee_link, EE_LINK);
  nh_.param<std::string>("planner_id", planner_id, PLANNER_ID);
  nh_.param<std::string>("reference_frame", reference_frame, REFERENCE_FRAME);
}

// TODO: Fix the state message to boolean
void DrawingManager::stateCallback(const std_msgs::String::ConstPtr& msg){
  ridgeback_state = msg->data;
}

void DrawingManager::visualizeStrokes(std::vector<Stroke> &strokes){
  int id = 0;
  for (int i = 0; i < strokes.size(); i++) { // storkes
    for (int j = 0; j < strokes[i].size(); j++) { // points
      marker.header.stamp = ros::Time::now();
      marker.id = id; id++;
      marker.points.push_back(strokes[i][j].position);
    }
    marker_pub.publish(marker);
    marker.points.erase(marker.points.begin());
  }
  ros::Duration(1.0).sleep();
}

int main(int argc, char **argv){
  //*********** Initialize ROS
  ros::init(argc, argv, "CommandRobotMoveit");
  ros::NodeHandle nh("~");

  DrawingManager dm(&nh);

  //*********** ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //*********** Init IIWA
  DrawingMoveit iiwa(nh, dm.move_group_name, dm.planner_id, dm.ee_link, dm.reference_frame);
  ros::Duration(1.0).sleep();

  //*********** init drawing pose
  dm.init_drawing_pose = iiwa.getCurrentPose().pose;
  dm.init_drawing_pose.position.x += 0.03;   // 3cm depper
  dm.init_drawing_pose.position.z += 0.05;  // move up

  //*********** Init Ridgeback (get drawing split ranges)
  boost::shared_ptr<std_msgs::Float64MultiArray const> drawing_ranges;

  //*********** Init Drawing
//  DrawingInput drawing_c(wall_file_name, drawing_file_name, 'c', init_drawing_pose, wall_pose);
//  DrawingInput drawing_m(wall_file_name, drawing_file_name, 'm', init_drawing_pose, wall_pose);
//  DrawingInput drawing_y(wall_file_name, drawing_file_name, 'y', init_drawing_pose, wall_pose);
  DrawingInput drawing_k(dm.wall_file_name, dm.drawing_file_name, 'k', dm.init_drawing_pose, dm.wall_pose);

  // visualization
  dm.visualizeStrokes (drawing_k.strokes);

  //*********** Wait for ridgeback
  ROS_INFO("Waiting for drawing ranges ...");
//  while(ros::ok()){
  drawing_ranges = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/iiwa_ridgeback_communicaiton/drawing_range",nh);
//    init = false;
//  }
  std_msgs::Float64MultiArray ranges = *drawing_ranges;
  ROS_INFO("Got drawing_range from Ridgeback");

  //*********** Split drawing by ranges
  drawing_k.splitByRangeArb(ranges);

  //*********** Drawing and moving
  int range_num = drawing_k.strokes_by_range.size();
//  init = true;
  bool done = false;

  while(ros::ok() && !done){
    for(int i = 0; i <= range_num ; i++){
      // get ridgeback's position and orientation
      ROS_INFO("Waiting for ridgeback's position and orientation ...");
      boost::shared_ptr<geometry_msgs::Pose const> ridegeback_pose_;
      ridegeback_pose_ = ros::topic::waitForMessage<geometry_msgs::Pose>("/iiwa_ridgeback_communicaiton/ridgeback/pose",nh);
      geometry_msgs::Pose ridegeback_pose = *ridegeback_pose_;

      dm.iiwa_state.data = "1"; // message recieved
      dm.ir_pub.publish(dm.iiwa_state);

      // relocate drawing coordinate according to ridgeback's location
      ROS_INFO("Relocate drawing coordinate according to ridgeback's pose");
      drawing_k.relocateDrawingsArb(ridegeback_pose, i);

//      // visualize
//      id = 0;
//      for (int k = 0; k < drawing_k.strokes_by_range[i].size(); k++) {
//        for (int j = 0; j < drawing_k.strokes_by_range[i][k].size(); j++) {
//          dm.target_range_marker.header.stamp = ros::Time::now();
//          dm.target_range_marker.id = id; id++;
//          dm.target_range_marker.points.push_back(drawing_k.strokes_by_range[i][k][j].position);
//        }
//        dm.marker_pub.publish(dm.target_range_marker);
//        dm.target_range_marker.points.erase(dm.target_range_marker.points.begin());
//      }
//      ros::Duration(1.0).sleep();

      // iiwa draw (color: c, m, y, k)
      ROS_INFO("IIWA Drawing Start");
      iiwa.drawStrokes(nh, drawing_k, drawing_k.color, i);

      // finished iiwa drawing make ridgeback move
      std::cout << "\n\n\n\n IIWA DONE \n\n";
      dm.iiwa_state.data = "0";
      dm.ir_pub.publish(dm.iiwa_state);  // ridgeback moves

      // wait for ridgeback to finish moving
      ROS_INFO("Waiting for ridgeback to finish moving");
      boost::shared_ptr<std_msgs::String const> ridgeback_done;
      ridgeback_done = ros::topic::waitForMessage<std_msgs::String>("/iiwa_ridgeback_communicaiton/ridgeback/state",nh);

      std::cout << "\n\n\n\n RIDGEBACK MOVED \n\n";
      // iiwa_state.data = "1";
      // ir_pub.publish(iiwa_state);  // tell ridgeback to stop moving
    }

    done = true;
  }
}
