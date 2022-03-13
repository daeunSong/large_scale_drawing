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
  wall_sub_ = nh_.subscribe("/vive/wall", 10, &DrawingManager::wallCallback, this);
  iiwa_pose_sub_ = nh_.subscribe("/vive/iiwa", 1, &DrawingManager::iiwaCallback, this);
}

// Init publisher
void DrawingManager::initPublisher() {
  ir_pub = nh_.advertise<std_msgs::Int32>("/iiwa_ridgeback_communicaiton/iiwa/state", 10);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("/target_drawing", 100);
}

// Init marker for target drawing
void DrawingManager::initMarker() {
  marker.header.frame_id = "/vive_ridgeback";
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
}

void DrawingManager::initMoveGroup() {
  // dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh_.param<std::string>("move_group", move_group_name, PLANNING_GROUP);
  nh_.param<std::string>("ee_link", ee_link, EE_LINK);
  nh_.param<std::string>("planner_id", planner_id, PLANNER_ID);
  nh_.param<std::string>("reference_frame", reference_frame, REFERENCE_FRAME);
}

void DrawingManager::wallCallback(const geometry_msgs::Pose::ConstPtr& msg){
  wall_ = *msg;
}

void DrawingManager::iiwaCallback(const geometry_msgs::Pose::ConstPtr& msg){
  iiwa_ = *msg;
}

// TODO: Fix the state message to boolean
void DrawingManager::stateCallback(const std_msgs::Int32::ConstPtr& msg){
  ridgeback_state = msg->data;
}

void DrawingManager::publishState(int state){
  iiwa_state.data = state;
  ir_pub.publish(iiwa_state); // send iiwa state
}

void DrawingManager::visualizeStrokes(std::vector<Stroke> &strokes){
  int id = 0;
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
  // Loop with 100 Hz rate
  ros::Rate loop_rate(100);
  spinner.start();

  //*********** Init IIWA
  DrawingIIWA iiwa(nh, dm.ee_link, dm.reference_frame);
  ros::Duration(1.0).sleep();

  //*********** init drawing pose
  dm.init_drawing_pose = iiwa.getCurrentPose().poseStamped.pose;
//  dm.init_drawing_pose.position.x += 0.03;   // 3cm depper
  dm.init_drawing_pose.position.z += 0.05;  // move up
//  iiwa.moveTransportPose();

  //*********** Init Drawing
  for (int i = 0; i < dm.colors.size(); i++){
    char ch[1];
    strcpy(ch, dm.colors[i].c_str());
    ROS_INFO("Drawing init");
    DrawingInput drawing(dm.wall_file_name, dm.drawing_file_name, ch[0], dm.init_drawing_pose, dm.wall_);
    dm.drawings.push_back(drawing);
//    dm.visualizeStrokes(drawing.strokes);
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

  //*********** Drawing and moving
  dm.range_num = dm.drawings[0].strokes_by_range.size();
  bool done = false;

  int range_ = 3;
  int stroke_ = 0;

  while(ros::ok() && !done){
    char input;
    for(int i = range_; i <= dm.range_num ; i++){
      // get ridgeback's position and orientation
      ROS_INFO("Waiting for ridgeback's position and orientation ...");
      boost::shared_ptr<geometry_msgs::Pose const> ridegeback_pose_;
      ridegeback_pose_ = ros::topic::waitForMessage<geometry_msgs::Pose>("/iiwa_ridgeback_communicaiton/ridgeback/pose",nh);
      geometry_msgs::Pose ridegeback_pose = *ridegeback_pose_;

//      dm.publishState(1); // publish iiwa state WORKING

      for (int j = 0; j < dm.drawings.size(); j++){
        // relocate drawing coordinate according to ridgeback's location
        ROS_INFO("Relocate drawing coordinate according to iiwa's pose");
        dm.drawings[j].relocateDrawingsArb(ridegeback_pose, i);
//        dm.drawings[j].relocateDrawingsArb(dm.iiwa_, i);
//        dm.drawings[j].saveFile(dm.drawings[j].strokes_by_range[i], i);

        // iiwa start drawing
        ROS_INFO("IIWA Drawing Start");
        iiwa.moveInitPose();
//        dm.visualizeStrokes(dm.drawings[j].strokes_by_range[i]);
        iiwa.drawStrokes(nh, dm.drawings[j], i, stroke_);
//
//        ros::Duration(0.5).sleep();
//        dm.publishState(2);
//        ros::Duration(1.0).sleep();
//        iiwa.moveInitPose();
//        std::cout << "Change Color" << std::endl;
//        std::cin >> input;
        // wait for ridgeback to finish moving
//        ROS_INFO("Waiting for ridgeback's position and orientation ...");
//        boost::shared_ptr<geometry_msgs::Pose const> ridegeback_pose_;
//        ridegeback_pose_ = ros::topic::waitForMessage<geometry_msgs::Pose>("/iiwa_ridgeback_communicaiton/ridgeback/pose",nh);
//        geometry_msgs::Pose ridegeback_pose = *ridegeback_pose_;
      }

      // finished iiwa drawing make ridgeback move
      std::cout << "\n\nIIWA DONE MOVE RIDGEBACK TO NEXT\n\n";
      std::cin >> input;
//      dm.publishState(0); // publish iiwa state DONE
//
//      // wait for ridgeback to finish moving
//      ROS_INFO("Waiting for ridgeback to finish moving");
//      boost::shared_ptr<std_msgs::Int32 const> ridgeback_state;
//      ridgeback_state = ros::topic::waitForMessage<std_msgs::Int32>("/iiwa_ridgeback_communicaiton/ridgeback/state",nh);
//
//      std::cout << "\n\nRIDGEBACK MOVED\n\n";
    }

    done = true;
  }
}
