#include <ros/ros.h>
#include <ros/package.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sound_play/sound_play.h>

#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

// function declarations
bool moveToGoal(double xGoal, double yGoal);
char choose();
void reachResult(bool state);
void position_error_check();
void add_marker(double xMarker, double yMarker, bool Action);
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);

// declare the coordinates
double xInitial = 0.0, yInitial = 0.0;
double xStart = -5.0, yStart = 4.5;
double xEnd = 2.5, yEnd = -5.0;

// delcare variables
double robot_x, robot_y, marker_x, marker_y;
double position_error = 10;
std::string path_to_sounds;
nav_msgs::Odometry pose_msg;
visualization_msgs::Marker marker;

bool goalReached = false;

int main(int argc, char** argv){
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	ros::spinOnce();
	sound_play::SoundClient sc;

  // go to the picking up position
  goalReached = moveToGoal(xStart, yStart);
  reachResult(goalReached);
  ROS_INFO("Take a 5 secs break...");
  ros::Duration(5.0).sleep();
  ROS_INFO("Continue to work...");

  // go to the drop off position
  goalReached = moveToGoal(xEnd, yEnd);
  reachResult(goalReached);
  ROS_INFO("Complete task!");
  ROS_INFO("Exit.");

	return 0;
}

bool moveToGoal(double xGoal, double yGoal){
  ros::NodeHandle n;
  ros::Subscriber odom_subscriber;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/
	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal position [%3.2f, %3.2f]", xGoal, yGoal);

  // publish marker
  add_marker(xGoal, yGoal, true);
  while (marker_pub.getNumSubscribers() < 1){
    if (!ros::ok()){return 0;}
    sleep(1);}
  marker_pub.publish(marker);

  // send goal position
	ac.sendGoal(goal);

  ROS_INFO("Moving to goal position...");
	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    std::cout<<"|-------------------------------|"<<std::endl;
    std::cout<<"|You have reached the destination! "<<std::endl;
    std::cout<<"|Wait 5 secs for manipulating... "<<std::endl;
    std::cout<<"|-------------------------------|"<<std::endl;
    ros::Duration(5.0).sleep();

    // delete current marker
    add_marker(xGoal, yGoal, false);
    while (marker_pub.getNumSubscribers() < 1){
    if (!ros::ok()){return 0;}
    sleep(1);}
    marker_pub.publish(marker);

		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}
}

void reachResult(bool state){
  sound_play::SoundClient sc;
  path_to_sounds = ros::package::getPath("pick_objects") + "/sounds/";

  if (state){
    ROS_INFO("Congratulations!");
    ros::spinOnce();
    sc.playWave(path_to_sounds+"sucess.wav");
    ros::spinOnce();
  }else{
    ROS_INFO("Hard Luck!");
    sc.playWave(path_to_sounds+"fail.wav");
  }
}

void add_marker(double xMarker, double yMarker, bool Action){

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  if(Action){
    marker.action = visualization_msgs::Marker::ADD;
  }
  else{
    marker.action = visualization_msgs::Marker::DELETE;
  }
  
  // Set the pose, scan and color of the marker. 
  marker.pose.position.x = xMarker;
  marker.pose.position.y = yMarker;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration();
}
