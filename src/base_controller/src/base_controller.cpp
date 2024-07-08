/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * Release date: 27/06/2024
 * 
 * Modified by: Felipe Ferreira
 * Last modification date: 08/07/2024
 * New version:

*************************************/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_broadcaster.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Quaternion odom_quat;
static double PI = 3.14159265;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "base_controller");

  //tell the action client that we want to spin a thread by default
 
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  geometry_msgs::Quaternion SetTheta(double theta);
  
  // set up the frame parameters
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 0.5;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation = SetTheta(0);

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, Robot reached the goal");
  else
  {
    ROS_INFO("The base failed to reach first goal for some reason");
    return 0;
  }
  ros::Duration(5.0).sleep();
}

geometry_msgs::Quaternion SetTheta(double theta){
  theta = theta * ( PI / 180);  //Degrees to RADs
  odom_quat = tf::createQuaternionMsgFromYaw( theta );
  return odom_quat;
}