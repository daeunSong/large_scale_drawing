#include "drawing_manager.h"

DrawingManager::DrawingManager(ros::NodeHandle* nh):nh_(*nh) {
  // init param
  nh_.getParam("/wall_file_name", wall_file_name);
  nh_.getParam("/wall_pose", wall_pose);
  nh_.getParam("/drawing_file_name", drawing_file_name);
  nh_.getParam("/colors", colors);

  initSubscriber();
  initPublisher();
  initMarker();
  initMoveGroup();
}

// Init subscriber
void DrawingManager::initSubscriber() {
  ir_sub_ = nh_.subscribe("/iiwa_ridgeback_communicaiton/ridgeback/state", 10, &DrawingManager::stateCallback, this);
}

// Init publisher
void DrawingManager::initPublisher() {
  ir_pub = nh_.advertise<std_msgs::String>("/iiwa_ridgeback_communicaiton/iiwa/state", 10);
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
  marker.color.a = 0.8;
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

void DrawingManager::publishState(std::string state){
  iiwa_state.data = state;
  ir_pub.publish(iiwa_state); // send iiwa state
}

void DrawingManager::visualizeStrokes(std::vector<Stroke> &strokes, char color){
  int id = int(color) * int(color) * int(color);

  if(color == 'c'){
    marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0;   // cyan (0, 255, 255)
  }else if(color == 'm'){
    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 1.0;   // magenta (255, 0, 255)
  }else if(color == 'y'){
    marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;   // yellow (255, 255, 0)
  }else if(color == 'r'){
    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;   // red (255, 0, 0)
  }else if(color == 'o'){
    marker.color.r = 1.0; marker.color.g = 0.58; marker.color.b = 0.03;   // orange (255, 127, 14)
  }else if(color == 'p'){
    marker.color.r = 0.58; marker.color.g = 0.404; marker.color.b = 0.741;   // purple (148, 103, 189)
  }else if(color == 'i'){
    marker.color.r = 0.99; marker.color.g = 0.48; marker.color.b = 0.49;   // pink (148, 103, 189)
  }else if(color == 'k'){ // black
    marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 0.0;   // black (0, 0, 0)
  }else if(color == 'b'){
    marker.color.r = 0.12; marker.color.g = 0.46; marker.color.b = 0.70;   // pink (148, 103, 189)
  }else if(color == 'g'){
    marker.color.r = 0.17; marker.color.g = 0.62; marker.color.b = 0.17;   // pink (148, 103, 189)
  }else{ // black
    marker.color.r = 0.73; marker.color.g = 0.74; marker.color.b = 0.13;   // black (0, 0, 0)
  }

  for (int i = 0; i < strokes.size(); i++) { // storkes
    marker.header.stamp = ros::Time::now();
    marker.id = id; id++;
    for (int j = 0; j < strokes[i].size(); j++) { // points
      marker.points.push_back(strokes[i][j].position);
    }
    marker_pub.publish(marker);
    ros::Duration(0.05).sleep();
    marker.points.clear();
  }
}

int main(int argc, char **argv){
  //*********** Initialize ROS
  ros::init(argc, argv, "drawingManager");
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
//  dm.init_drawing_pose.position.x += 0.03;   // 3cm depper
//  dm.init_drawing_pose.position.z += 0.05;  // move up

  //*********** Init Drawing
  for (int i = 0; i < dm.colors.size(); i++){
    char ch[1];
    strcpy(ch, dm.colors[i].c_str());
    ROS_INFO("Drawing init");
    DrawingInput drawing(dm.wall_file_name, dm.drawing_file_name, ch[0], dm.init_drawing_pose, dm.wall_pose);
    dm.drawings.push_back(drawing);
//    dm.visualizeStrokes(drawing.strokes, drawing.color);
  }

  //*********** Wait for ridgeback
  ROS_INFO("Waiting for drawing ranges ...");
  boost::shared_ptr<std_msgs::Float64MultiArray const> drawing_ranges;
  drawing_ranges = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/iiwa_ridgeback_communicaiton/drawing_range",nh);
  dm.ranges = *drawing_ranges;
  ROS_INFO("Got drawing_range from Ridgeback");

  //*********** Split drawing by ranges
  for (int i = 0; i < dm.colors.size(); i++){
    dm.drawings[i].splitByRangeArb(dm.ranges);
  }

//  dm.range_num = dm.drawings[0].strokes_by_range.size();
//  dm.visualizeStrokes(dm.drawings[0].strokes_by_range[0], 'r');
//  dm.visualizeStrokes(dm.drawings[1].strokes_by_range[0], 'r');
//  dm.visualizeStrokes(dm.drawings[2].strokes_by_range[0], 'r');
//
//  dm.visualizeStrokes(dm.drawings[0].strokes_by_range[1], 'o');
//  dm.visualizeStrokes(dm.drawings[1].strokes_by_range[1], 'o');
//  dm.visualizeStrokes(dm.drawings[2].strokes_by_range[1], 'o');
//
//  dm.visualizeStrokes(dm.drawings[0].strokes_by_range[2], 'p');
//  dm.visualizeStrokes(dm.drawings[1].strokes_by_range[2], 'p');
//  dm.visualizeStrokes(dm.drawings[2].strokes_by_range[2], 'p');
//
//  dm.visualizeStrokes(dm.drawings[0].strokes_by_range[3], 'b');
//  dm.visualizeStrokes(dm.drawings[1].strokes_by_range[3], 'b');
//  dm.visualizeStrokes(dm.drawings[2].strokes_by_range[3], 'b');
//
//  dm.visualizeStrokes(dm.drawings[0].strokes_by_range[4], 'g');
//  dm.visualizeStrokes(dm.drawings[1].strokes_by_range[4], 'g');
//  dm.visualizeStrokes(dm.drawings[2].strokes_by_range[4], 'g');

  //*********** Drawing and moving
  dm.range_num = dm.drawings[0].strokes_by_range.size();
  bool done = false;

  while(ros::ok() && !done){
    for(int i = 0; i <= dm.range_num ; i++){
      // get ridgeback's position and orientation
      ROS_INFO("Waiting for ridgeback's position and orientation ...");
      boost::shared_ptr<geometry_msgs::Pose const> ridegeback_pose_;
      ridegeback_pose_ = ros::topic::waitForMessage<geometry_msgs::Pose>("/iiwa_ridgeback_communicaiton/ridgeback/pose",nh);
      geometry_msgs::Pose ridegeback_pose = *ridegeback_pose_;

      dm.publishState("1"); // publish iiwa state WORKING

      for (int j = 0; j < dm.drawings.size(); j++){
        // relocate drawing coordinate according to ridgeback's location
        ROS_INFO("Relocate drawing coordinate according to ridgeback's pose");
        dm.drawings[j].relocateDrawingsArb(ridegeback_pose, i);

        // iiwa start drawing
        ROS_INFO("IIWA Drawing Start");
        iiwa.drawStrokes(nh, dm.drawings[j], i);
      }

      // finished iiwa drawing make ridgeback move
      std::cout << "\n\nIIWA DONE\n\n";
      dm.publishState("0"); // publish iiwa state DONE

      // wait for ridgeback to finish moving
      ROS_INFO("Waiting for ridgeback to finish moving");
      boost::shared_ptr<std_msgs::String const> ridgeback_state;
      ridgeback_state = ros::topic::waitForMessage<std_msgs::String>("/iiwa_ridgeback_communicaiton/ridgeback/state",nh);

      std::cout << "\n\nRIDGEBACK MOVED\n\n";
    }

    done = true;
  }
}
