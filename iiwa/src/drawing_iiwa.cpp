#include <ros/ros.h>
#include <ros/package.h>

#include <actionlib/client/simple_action_client.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>
#include <iiwa_msgs/MoveAlongSplineAction.h>
#include <iiwa_msgs/SetPTPJointSpeedLimits.h>
#include <iiwa_msgs/SetPTPCartesianSpeedLimits.h>
#include <iiwa_msgs/SetSmartServoLinSpeedLimits.h>
#include <iiwa_msgs/SetEndpointFrame.h>
#include <iiwa_ros/command/cartesian_pose_linear.hpp>
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/service/control_mode.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>
#include <iiwa_ros/iiwa_ros.hpp>
#include <iiwa_ros/conversions.hpp>


#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define _USE_MATH_DEFINES
#define TXT_FILE "/input/heart/heart_path_k.txt"
#define BACKWARD 0.05
#define TRANSLATE_UP 0.48
#define TARGET_SIZE 0.5

using namespace std;

// getTimeToDestination() can also return negative values and the info from the cabinet take some milliseconds to update once the motion is started.
// That means that if you call getTimeToDestination() right after you set a target pose, you might get the wrong info (e.g. a negative number).
// This function tried to call getTimeToDestination() until something meaningful is obtained or until a maximum amount of time passed.
void sleepForMotion(iiwa_ros::service::TimeToDestinationService& iiwa, const double maxSleepTime) {
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

static iiwa_msgs::SplineSegment getSplineSegment (geometry_msgs::Pose waypoint_pose, int type = iiwa_msgs::SplineSegment::SPL) {
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

static bool setPTPJointSpeedLimits(ros::NodeHandle& nh) {
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

static bool setPTPCartesianSpeedLimits(ros::NodeHandle& nh) {
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

static bool setSmartServoLinSpeedLimits(ros::NodeHandle& nh) {
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

static bool setEndpointFrame(ros::NodeHandle& nh, std::string frameId = "iiwa_link_ee") {
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

vector<string> split(string input, char delimiter){
    vector<string> ans;
    stringstream str(input);
    string temp;

    while(getline(str, temp, delimiter)){
        ans.push_back(temp);
    }

    return ans;
}


//vector<double> calculateNormal(vector<geometry_msgs::Pose> P){
//    vector<double> normal;
//    vector<double> P1, P2;
//
//    // get 2 vectors from 3 points
//    P1.push_back(P[0].position.x - P[1].position.x);
//    P1.push_back(P[0].position.y - P[1].position.y);
//    P1.push_back(P[0].position.z - P[1].position.z);
//    P2.push_back(P[0].position.x - P[2].position.x);
//    P2.push_back(P[0].position.y - P[2].position.y);
//    P2.push_back(P[0].position.z - P[2].position.z);
//
//    // calculate normal vector
//    normal.push_back(P1[1]*P2[2] - P1[2]*P2[1]);
//    normal.push_back(P1[2]*P2[0] - P1[0]*P2[2]);
//    normal.push_back(P1[0]*P2[1] - P1[1]*P2[0]);
//
//    // make x of normal vector as 1
//    normal[1] = normal[1]/normal[0];
//    normal[2] = normal[2]/normal[0];
//    normal[0] = 1;
//
//    // find d (x + y + z + d = 0)
//    double d = -1*P[0].position.x - normal[1]*P[0].position.y - normal[2]*P[0].position.z;
//    normal.push_back(d);
//
//    return normal;
//}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "CommandRobotIIWA");
	ros::NodeHandle nh;

	// Set speed limit for motions in joint coordinates
	if (!setPTPJointSpeedLimits(nh)) {
		return 1;
	}

	// Set speed limits for motions in cartesian coordinates
	if (!setPTPCartesianSpeedLimits(nh)) {
		return 1;
	}

	if (!setSmartServoLinSpeedLimits(nh)) {
		return 1;
	}

	// Set endpoint frame to flange, so that our Cartesian target coordinates are tool independent
	if (!setEndpointFrame(nh)) {
		return 1;
	}


iiwa_ros::command::CartesianPoseLinear iiwa_pose_command;
  iiwa_ros::state::CartesianPose iiwa_pose_state;

	// for Cartesian Impedance Control
  iiwa_ros::service::ControlModeService iiwa_control_mode;
  iiwa_ros::service::TimeToDestinationService iiwa_time_destination;
  // Low stiffness only along Z.
  iiwa_msgs::CartesianQuantity cartesian_stiffness = iiwa_ros::conversions::CartesianQuantityFromFloat(1500,1500,350,300,300,300);
  iiwa_msgs::CartesianQuantity cartesian_damping = iiwa_ros::conversions::CartesianQuantityFromFloat(0.7);

  std::vector<geometry_msgs::Pose> drawing_stroke;
  geometry_msgs::Pose drawing_point;

	iiwa_pose_command.init("iiwa");
  iiwa_pose_state.init("iiwa");
  iiwa_control_mode.init("iiwa");
	iiwa_time_destination.init("iiwa");

	// Create the action clients
	// Passing "true" causes the clients to spin their own threads
	actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction> jointPositionClient("/iiwa/action/move_to_joint_position", true);
	actionlib::SimpleActionClient<iiwa_msgs::MoveAlongSplineAction> splineMotionClient("/iiwa/action/move_along_spline", true);


	ROS_INFO("Waiting for action servers to start...");
	// Wait for the action servers to start
	jointPositionClient.waitForServer(); //will wait for infinite time
	splineMotionClient.waitForServer();

	// ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();
	ROS_INFO("Spinner started...");

	ROS_INFO("Action server started, moving to start pose...");
	// Define a goal
	iiwa_msgs::MoveToJointPositionGoal jointPositionGoal;
	jointPositionGoal.joint_position.position.a1 =  0.00;
	jointPositionGoal.joint_position.position.a2 =  0.435332;
	jointPositionGoal.joint_position.position.a3 =  0.00;
	jointPositionGoal.joint_position.position.a4 = -1.91986;
	jointPositionGoal.joint_position.position.a5 =  0.00;
	jointPositionGoal.joint_position.position.a6 = -0.785399;
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

  ros::Duration(3).sleep(); // wait for 3 sec

  // Get Current Position
	iiwa_msgs::CartesianPose wall_pose, init_pose;
  iiwa_msgs::CartesianPose command_cartesian_position;
  init_pose = iiwa_pose_state.getPose();
	command_cartesian_position = init_pose;

  // Move forward to detect the wall
  ROS_INFO("Start detecting the wall");
	iiwa_msgs::MoveAlongSplineGoal splineMotion;
  splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.poseStamped.pose, iiwa_msgs::SplineSegment::LIN));
  command_cartesian_position.poseStamped.pose.position.x += BACKWARD;
  splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.poseStamped.pose, iiwa_msgs::SplineSegment::LIN));

	// Execute motion
  splineMotionClient.sendGoal(splineMotion);
  splineMotionClient.waitForResult();
	splineMotion.spline.segments.clear();

  ros::Duration(3).sleep(); // wait for 3 sec
  wall_pose = iiwa_pose_state.getPose();

  ROS_INFO("Moving Backward ... ");
	command_cartesian_position = wall_pose;
  double x = wall_pose.poseStamped.pose.position.x;//0.478509765292;  // DEPTH
	double y = wall_pose.poseStamped.pose.position.y;//0;
	double z = wall_pose.poseStamped.pose.position.z;//0.613500561539;  // HEIGHT
	drawing_point = wall_pose.poseStamped.pose;

	iiwa_control_mode.setPositionControlMode();
	command_cartesian_position.poseStamped.pose.position.x -= BACKWARD;
	iiwa_pose_command.setPose(command_cartesian_position.poseStamped);
	sleepForMotion(iiwa_time_destination, 2.0);
	ros::Duration(0.2).sleep();

  // TXT file with list of coordinates
//  // get 3 points on wall and calculate normal vector  ---------------------------------------------------------------------------------
//  ROS_INFO("Detecting 3 points on wall ... ");
//  vector<double> normal;
//  vector<geometry_msgs::Pose> P;
//
//  P.push_back(wall_pose.poseStamped.pose);
//
//  // get one point above the wall_pose
//  command_cartesian_position = iiwa_pose_state.getPose();
//  splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.poseStamped.pose, iiwa_msgs::SplineSegment::LIN));
//  command_cartesian_position.poseStamped.pose.position.x += BACKWARD;
//  command_cartesian_position.poseStamped.pose.position.z += 0.05;
//  splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.poseStamped.pose, iiwa_msgs::SplineSegment::LIN));
//  // Execute motion
//  splineMotionClient.sendGoal(splineMotion);
//  splineMotionClient.waitForResult();
//	splineMotion.spline.segments.clear();
//  // wait for 3 sec
//  ros::Duration(3).sleep();
//  command_cartesian_position = iiwa_pose_state.getPose();
//
//  P.push_back(command_cartesian_position.poseStamped.pose);
//
//  // Move Backward
//  ROS_INFO("Moving Backward ... ");
//	iiwa_control_mode.setPositionControlMode();
//	command_cartesian_position.poseStamped.pose.position.x -= BACKWARD;
//	iiwa_pose_command.setPose(command_cartesian_position.poseStamped);
//	sleepForMotion(iiwa_time_destination, 2.0);
//	ros::Duration(0.2).sleep();
//
//
//  // get one point on the right of the wall pose
//  splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.poseStamped.pose, iiwa_msgs::SplineSegment::LIN));
//  command_cartesian_position = wall_pose;
//  command_cartesian_position.poseStamped.pose.position.x += 0.02;
//  command_cartesian_position.poseStamped.pose.position.y += 0.05;
//  splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.poseStamped.pose, iiwa_msgs::SplineSegment::LIN));
//  // Execute motion
//  splineMotionClient.sendGoal(splineMotion);
//  splineMotionClient.waitForResult();
//	splineMotion.spline.segments.clear();
//  // wait for 3 sec
//  ros::Duration(3).sleep();
//  command_cartesian_position = iiwa_pose_state.getPose();
//
//  P.push_back(command_cartesian_position.poseStamped.pose);
//
//  // Move Backward
//  ROS_INFO("Moving Backward ... ");
//	iiwa_control_mode.setPositionControlMode();
//	command_cartesian_position.poseStamped.pose.position.x -= BACKWARD;
//	iiwa_pose_command.setPose(command_cartesian_position.poseStamped);
//	sleepForMotion(iiwa_time_destination, 2.0);
//	ros::Duration(0.2).sleep();


//  normal = calculateNormal(P);
//  // calculate angle between wall and ridgeback
//  double ang = acos(normal[0] / sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]+normal[2]));
//  double y_formula;  // save b from y = az + b as a is 0
//                     // a = -1*normal[2]/normal[1];
//  y_formula = -1*(normal[3])/normal[1];


  ifstream txt(ros::package::getPath("large_scale_drawing")+TXT_FILE);
  // check if text file is well opened
  if(!txt.is_open()) {
    cout << "FILE NOT FOUND" << endl;
    return 1;
  }

  string line;
  int stroke_num = 0;
  bool ready_to_draw = false;

  getline(txt, line); // drawing size
  vector<string> tempSplit_ = split(line, ' ');
  double width = stod(tempSplit_[0]);
  double height = stod(tempSplit_[1]);
  double ratio = width / height;

  while (getline(txt, line)) {
    if (line == "End") {
      stroke_num ++;

      // move forward to the first drawing position
      ROS_INFO("Moving Forward ... ");
      splineMotion.spline.segments.push_back(getSplineSegment(drawing_stroke[0], iiwa_msgs::SplineSegment::LIN));

      // Execute motion
      splineMotionClient.sendGoal(splineMotion);
      splineMotionClient.waitForResult();
      splineMotion.spline.segments.clear();
      // ros::Duration(0.3).sleep();

      // draw a stroke
      ROS_INFO("Drawing %d th stroke ...", stroke_num);
      for (int i = 0 ; i < drawing_stroke.size(); i++)
          splineMotion.spline.segments.push_back(getSplineSegment(drawing_stroke[i], iiwa_msgs::SplineSegment::SPL));

      ROS_INFO("Executing motion ... ");
      // Execute motion
      splineMotionClient.sendGoal(splineMotion);
      splineMotionClient.waitForResult();
      splineMotion.spline.segments.clear();
      //ros::Duration(0.5).sleep();

      ROS_INFO("Move Backward");
      iiwa_control_mode.setPositionControlMode();
      command_cartesian_position.poseStamped.pose = drawing_stroke.back();
      command_cartesian_position.poseStamped.pose.position.x -= BACKWARD;
      iiwa_pose_command.setPose(command_cartesian_position.poseStamped);
      sleepForMotion(iiwa_time_destination, 2.0);
      ros::Duration(0.5).sleep();

      ready_to_draw = false;
      drawing_stroke.clear();
    }
		else {
			// read drawing
			vector<string> tempSplit = split(line, ' ');
      y = (stod(tempSplit[0])-0.5) * ratio * TARGET_SIZE;
      z = (-stod(tempSplit[1])+0.5) * TARGET_SIZE + TRANSLATE_UP;



//      if(ang != 0){ // if ridgeback is not parallel to the wall
//        double radius = abs(y_formula - y);
//        // cout << "CALCULATING: " << ang << " " << sin(ang) << " " << cos(ang) << " " << radius << endl;
//        x = radius * sin(ang); //+ init_cartesian_position.pose.position.x;
//        y = radius * cos(ang) + y_formula;
//      }

			if (!ready_to_draw) {
				// move to the ready position (off the wall)
				ROS_INFO("Moving To Ready Position ... ");
				iiwa_control_mode.setPositionControlMode();
				command_cartesian_position.poseStamped.pose.position.x = x - BACKWARD;
				command_cartesian_position.poseStamped.pose.position.y = y;
				command_cartesian_position.poseStamped.pose.position.z = z;

				iiwa_pose_command.setPose(command_cartesian_position.poseStamped);
				sleepForMotion(iiwa_time_destination, 2.0);
				ready_to_draw = true;
				ros::Duration(1.0).sleep();

				splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.poseStamped.pose, iiwa_msgs::SplineSegment::LIN));
			}

      drawing_point.position.x = x + 0.002;
      drawing_point.position.y = y;
      drawing_point.position.z = z;
      drawing_stroke.push_back(drawing_point); // push the point
		}
  }

	ROS_INFO("Moving To Init Position ... ");
	iiwa_control_mode.setPositionControlMode();
	iiwa_pose_command.setPose(init_pose.poseStamped);
	sleepForMotion(iiwa_time_destination, 2.0);

	spinner.stop();
	ROS_INFO("Done.");

	//exit
	return 0;
}
