#include "drawing_iiwa.h"

DrawingIIWA::DrawingIIWA(ros::NodeHandle &nh, std::string ee_link_, std::string reference_frame)
    : jointPositionClient("/iiwa/action/move_to_joint_position", true),
      cartesianPositionClient("/iiwa/action/MoveToCartesianPose", true),
      splineMotionClient("/iiwa/action/move_along_spline", true)
{
  ROS_INFO("Waiting for action servers to start...");
  // Wait for the action servers to start
  jointPositionClient.waitForServer(); //will wait for infinite time
  splineMotionClient.waitForServer();
  int state = DrawingIIWA::init(nh, ee_link_);

  // Move robot to the initial pose
  this->moveInitPose();
  this->init_pose = this->getCurrentPose();
}

int DrawingIIWA::init(ros::NodeHandle &nh, std::string ee_link_){
  // Set speed limit for motions in joint coordinates
  if (!this->setPTPJointSpeedLimits(nh))
    return 0;
  // Set speed limits for motions in cartesian coordinates
  if (!this->setPTPCartesianSpeedLimits(nh))
    return 0;
  if (!this->setSmartServoLinSpeedLimits(nh))
    return 0;
  // Set endpoint frame to flange, so that our Cartesian target coordinates are tool independent
  if (!this->setEndpointFrame(nh, ee_link_))
    return 0;

  // Low stiffness only along Z.
  this->cartesian_stiffness = iiwa_ros::conversions::CartesianQuantityFromFloat(1500,1500,350,300,300,300);
  this->cartesian_damping = iiwa_ros::conversions::CartesianQuantityFromFloat(0.7);

  this->iiwa_pose_command.init("iiwa");
  this->iiwa_pose_state.init("iiwa");
  this->iiwa_control_mode.init("iiwa");
  this->iiwa_time_destination.init("iiwa");

  return 1;
}

int DrawingIIWA::moveInitPose( ){
  iiwa_msgs::MoveToJointPositionGoal jointPositionGoal;
  jointPositionGoal.joint_position.position.a1 =  0.00;
  jointPositionGoal.joint_position.position.a2 =  0.3228859; // 18.5d// 0.435332;
  jointPositionGoal.joint_position.position.a3 =  0.00;
    jointPositionGoal.joint_position.position.a4 = -2.05948852; // -118d // -1.91986;
  jointPositionGoal.joint_position.position.a5 =  0.00;
  jointPositionGoal.joint_position.position.a6 = -0.759218225;// -43.5// -0.785399;
  jointPositionGoal.joint_position.position.a7 = -0.523599;
  // Send goal to action server
  jointPositionClient.sendGoal(jointPositionGoal);

  // Wait for the action to finish
  bool finished_before_timeout = jointPositionClient.waitForResult(ros::Duration(60.0));

  if (!finished_before_timeout) {
    ROS_WARN("iiwa motion timed out - exiting...");
    return 0;
  }
  else if (!jointPositionClient.getResult()->success) {
    ROS_ERROR("Action execution failed - exiting...");
    return 0;
  }
  return 1;
}

int DrawingIIWA::moveTransportPose(){
  iiwa_msgs::MoveToJointPositionGoal jointPositionGoal;
  jointPositionGoal.joint_position.position.a1 =  0.00;
  jointPositionGoal.joint_position.position.a2 =  0.435332;
  jointPositionGoal.joint_position.position.a3 =  0.00;
  jointPositionGoal.joint_position.position.a4 = -1.91986;
  jointPositionGoal.joint_position.position.a5 =  0.00;
  jointPositionGoal.joint_position.position.a6 = 0.785398;//-0.785399;
  jointPositionGoal.joint_position.position.a7 = -0.523599;
  // Send goal to action server
  jointPositionClient.sendGoal(jointPositionGoal);

  // Wait for the action to finish
  bool finished_before_timeout = jointPositionClient.waitForResult(ros::Duration(60.0));

  if (!finished_before_timeout) {
    ROS_WARN("iiwa motion timed out - exiting...");
    return 0;
  }
  else if (!jointPositionClient.getResult()->success) {
    ROS_ERROR("Action execution failed - exiting...");
    return 0;
  }
  return 1;
}

iiwa_msgs::CartesianPose DrawingIIWA::getCurrentPose(){
  return this->iiwa_pose_state.getPose();
}

// getTimeToDestination() can also return negative values and the info from the cabinet take some milliseconds to update once the motion is started.
// That means that if you call getTimeToDestination() right after you set a target pose, you might get the wrong info (e.g. a negative number).
// This function tried to call getTimeToDestination() until something meaningful is obtained or until a maximum amount of time passed.
void DrawingIIWA::sleepForMotion(iiwa_ros::service::TimeToDestinationService& iiwa, const double maxSleepTime) {
  double ttd = iiwa.getTimeToDestination();
  ros::Time start_wait = ros::Time::now();
  while (ttd < 0.0 && (ros::Time::now() - start_wait) < ros::Duration(maxSleepTime)) {
    ros::Duration(0.5).sleep();
    ttd = iiwa.getTimeToDestination();
  }
  if (ttd > 0.0) {
    ROS_INFO_STREAM("Sleeping for " << ttd << " seconds.");
    ros::Duration(ttd).sleep();
  }
}

iiwa_msgs::SplineSegment DrawingIIWA::getSplineSegment (geometry_msgs::Pose waypoint_pose, int type=iiwa_msgs::SplineSegment::SPL) {
  iiwa_msgs::SplineSegment segment;
  // Segment type
  segment.type = type;
  // Header
  segment.point.poseStamped.header.frame_id = "iiwa_link_0";
  // Pose
  segment.point.poseStamped.pose.position.x = waypoint_pose.position.x;
  segment.point.poseStamped.pose.position.y = waypoint_pose.position.y;
  segment.point.poseStamped.pose.position.z = waypoint_pose.position.z;
  // Orientation
  segment.point.poseStamped.pose.orientation.x = waypoint_pose.orientation.x;
  segment.point.poseStamped.pose.orientation.y = waypoint_pose.orientation.y;
  segment.point.poseStamped.pose.orientation.z = waypoint_pose.orientation.z;
  segment.point.poseStamped.pose.orientation.w = waypoint_pose.orientation.w;
  // Redundancy
  segment.point.redundancy.status = -1;
  segment.point.redundancy.turn = -1;

  return segment;
}

bool DrawingIIWA::setPTPJointSpeedLimits(ros::NodeHandle& nh) {
  ROS_INFO("Setting PTP joint speed limits...");
  ros::ServiceClient setPTPJointSpeedLimitsClient = nh.serviceClient<iiwa_msgs::SetPTPJointSpeedLimits>("/iiwa/configuration/setPTPJointLimits");
  iiwa_msgs::SetPTPJointSpeedLimits jointSpeedLimits;
  jointSpeedLimits.request.joint_relative_velocity = 0.2;
  jointSpeedLimits.request.joint_relative_acceleration = 0.5;
  if (!setPTPJointSpeedLimitsClient.call(jointSpeedLimits)) {
    ROS_ERROR("Service call failed.");
    return false;
  }
  else if (!jointSpeedLimits.response.success) {
    ROS_ERROR_STREAM("Service call returned error: "+jointSpeedLimits.response.error);
    return false;
  }

  ROS_INFO("Done.");
  return true;
}

bool DrawingIIWA::setPTPCartesianSpeedLimits(ros::NodeHandle& nh) {
  ROS_INFO("Setting PTP Cartesian speed limits...");
  ros::ServiceClient setPTPCartesianSpeedLimitsClient = nh.serviceClient<iiwa_msgs::SetPTPCartesianSpeedLimits>("/iiwa/configuration/setPTPCartesianLimits");
  iiwa_msgs::SetPTPCartesianSpeedLimits cartesianSpeedLimits;
  cartesianSpeedLimits.request.maxCartesianVelocity = 0.5;
  cartesianSpeedLimits.request.maxCartesianAcceleration = 0.5;
  cartesianSpeedLimits.request.maxCartesianJerk = -1.0; // ignore
  cartesianSpeedLimits.request.maxOrientationVelocity = 0.5;
  cartesianSpeedLimits.request.maxOrientationAcceleration = 0.5;
  cartesianSpeedLimits.request.maxOrientationJerk = -1.0; // ignore
  if (!setPTPCartesianSpeedLimitsClient.call(cartesianSpeedLimits)) {
    ROS_ERROR("Failed.");
    return false;
  }
  else if (!cartesianSpeedLimits.response.success) {
    ROS_ERROR_STREAM("Service call returned error: "+cartesianSpeedLimits.response.error);
    return false;
  }

  ROS_INFO("Done.");
  return true;
}

bool DrawingIIWA::setSmartServoLinSpeedLimits(ros::NodeHandle& nh) {
  ROS_INFO("Setting SmartServo Cartesian speed limits...");
  ros::ServiceClient setSmartServoLinSpeedLimitsClient = nh.serviceClient<iiwa_msgs::SetSmartServoLinSpeedLimits>("/iiwa/configuration/setSmartServoLinLimits");
  iiwa_msgs::SetSmartServoLinSpeedLimits smartServoLinLimits;

  smartServoLinLimits.request.max_cartesian_velocity.linear.x = 0.3;
  smartServoLinLimits.request.max_cartesian_velocity.linear.y = 0.3;
  smartServoLinLimits.request.max_cartesian_velocity.linear.z = 0.3;
  smartServoLinLimits.request.max_cartesian_velocity.angular.x = M_PI/10;
  smartServoLinLimits.request.max_cartesian_velocity.angular.y = M_PI/10;
  smartServoLinLimits.request.max_cartesian_velocity.angular.z = M_PI/10;
  if (!setSmartServoLinSpeedLimitsClient.call(smartServoLinLimits)) {
    ROS_ERROR("Failed.");
    return false;
  }
  else if (!smartServoLinLimits.response.success) {
    ROS_ERROR_STREAM("Service call returned error: "+smartServoLinLimits.response.error);
    return false;
  }

  ROS_INFO("Done.");
  return true;
}

bool DrawingIIWA::setEndpointFrame(ros::NodeHandle& nh, std::string frameId="iiwa_link_ee") {
  ROS_INFO_STREAM("Setting endpoint frame to \""<<frameId<<"\"...");
  ros::ServiceClient setEndpointFrameClient = nh.serviceClient<iiwa_msgs::SetEndpointFrame>("/iiwa/configuration/setEndpointFrame");
  iiwa_msgs::SetEndpointFrame endpointFrame;
  endpointFrame.request.frame_id = frameId;
  if (!setEndpointFrameClient.call(endpointFrame)) {
    ROS_ERROR("Failed.");
    return false;
  }
  else if (!endpointFrame.response.success) {
    ROS_ERROR_STREAM("Service call returned error: "+endpointFrame.response.error);
    return false;
  }

  ROS_INFO("Done.");
  return true;
}

void DrawingIIWA::drawStrokes(ros::NodeHandle &nh, DrawingInput &drawing_strokes, int range_num){

  // drawing commands related
  std_msgs::Bool ready;
  bool finished_before_timeout;
  int j = 0;

  iiwa_msgs::CartesianPose command_cartesian_position;
  command_cartesian_position = this->init_pose;
  iiwa_msgs::MoveAlongSplineGoal splineMotion;
  int num_strokes = drawing_strokes.strokes_by_range[range_num].size();

  for (auto stroke : drawing_strokes.strokes_by_range[range_num]) {
//    iiwa_control_mode.setPositionControlMode();
//    command_cartesian_position.poseStamped.pose = stroke[0];
//    command_cartesian_position.poseStamped.pose.position.x -= BACKWARD/2;
//    setEndpointFrame(nh, "tool_ready_link_ee");

    // move to ready position in position control
//    iiwa_pose_command.setPose(command_cartesian_position.poseStamped);
//    ros::Duration(0.5).sleep();
//    sleepForMotion(iiwa_time_destination, 5.0);
//    ros::Duration(0.5).sleep();
    setEndpointFrame(nh, "tool_ready_link_ee");
    splineMotion.spline.segments.push_back(getSplineSegment(stroke[0], iiwa_msgs::SplineSegment::LIN));
    splineMotionClient.sendGoal(splineMotion);
//    splineMotionClient.waitForResult();

    // Wait for the action to finish
    finished_before_timeout = splineMotionClient.waitForResult();
    if (!finished_before_timeout) {
      ROS_WARN("iiwa motion timed out - exiting...");
    }
    else if (!jointPositionClient.getResult()->success) {
      ROS_ERROR("Action execution failed - exiting...");
    }
    splineMotion.spline.segments.clear();

//    // draw
//    setEndpointFrame(nh, "tool_link_ee");
    std::cout << "Drawing " << drawing_strokes.color << " " << range_num << "th range, " << j << "th stroke out of " << num_strokes << " strokes ... " << std::endl;
//
//    splineMotion.spline.segments.push_back(this->getSplineSegment(stroke[0], iiwa_msgs::SplineSegment::LIN));
//    for (int j = 1 ; j < stroke.size(); j++)
//      splineMotion.spline.segments.push_back(this->getSplineSegment(stroke[j], iiwa_msgs::SplineSegment::SPL));
//    splineMotionClient.sendGoal(splineMotion);
//    splineMotionClient.waitForResult();
//    splineMotion.spline.segments.clear();
//
    // move backward
//    setEndpointFrame(nh, "tool_back_link_ee");
//    iiwa_control_mode.setPositionControlMode();
//    command_cartesian_position.poseStamped.pose = stroke.back();
////    command_cartesian_position.poseStamped.pose.position.x -= BACKWARD;
//    iiwa_pose_command.setPose(command_cartesian_position.poseStamped);
//    ros::Duration(0.5).sleep();
//    sleepForMotion(iiwa_time_destination, 3.0);
//    ros::Duration(0.5).sleep();
    j++;
  }
  moveTransportPose();

}

