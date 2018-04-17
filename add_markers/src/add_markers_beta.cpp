#include <ros/ros.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
using namespace std;

double robot_x, robot_y, marker_x, marker_y;
double position_error = 10;
nav_msgs::Odometry pose_msg;
visualization_msgs::Marker marker;

double xStart = -5.0, yStart = 4.5;
double xEnd = 2.5, yEnd = -5.0;

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  pose_msg = *odom_msg;
  robot_x = pose_msg.pose.pose.position.x;
  robot_y = pose_msg.pose.pose.position.y;
}

void position_error_check(){
    position_error = pow(robot_x - marker_x,2) + pow(robot_y - marker_y,2);
    ROS_INFO("The position error, %5.2f", position_error);
    ROS_INFO("The robot position, %5.2f, %5.2f", robot_x, robot_y);
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
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  marker_x = marker.pose.position.x;
  marker_y = marker.pose.position.y;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  bool end_marker = false;
  bool initial_marker = true;
  bool startReached = false;
  bool endReached = false;

  ros::Subscriber odom_subscriber;
  odom_subscriber = n.subscribe("/odom", 10, odom_callback);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);



  // add marker at start point
  add_marker(xStart, yStart, true);
  while (marker_pub.getNumSubscribers() < 1){
      if (!ros::ok()){return 0;}
      sleep(1);}
  marker_pub.publish(marker);
  
  // wait for 5 secs
  ros::Duration(5.0).sleep();
  // delete marker
  add_marker(xStart, yStart, false);
  while (marker_pub.getNumSubscribers() < 1){
      if (!ros::ok()){return 0;}
      sleep(1);}
  marker_pub.publish(marker);

  // add marker at drop off point
  add_marker(xEnd, yEnd, true);
  while (marker_pub.getNumSubscribers() < 1){
    if (!ros::ok()){return 0;}
    sleep(1);}
  marker_pub.publish(marker);

  ros::spinOnce();
  ros::Duration(0.5).sleep();
    

  return 0;





}

