/************************************
 *  Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * Release date: 27/06/2024
 * 
 * Modified by: Felipe Ferreira
 * Last modification date: 
 * New version:

*************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

#include "base_controller/move_goal.h"
#include "odometry/pose_odom.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"main");
    ros::NodeHandle nh;

    ROS_INFO("main node is now started");

    ros::ServiceClient move_goal    = nh.serviceClient<base_controller::move_goal>("base_controller/goal");
    ros::ServiceClient set_position = nh.serviceClient<odometry::pose_odom>("odometry/set_position");

    ros::Duration(5).sleep();

    odometry::pose_odom pose;

    pose.request.x = 0.3;
    pose.request.y = 0.3;
    pose.request.th = 90;

    set_position.call( pose );

    base_controller::move_goal goal;

    goal.request.x = 0.3;
    goal.request.y = 1.0;
    goal.request.th = 90;

    move_goal.call( goal );


    ros::spin();
    ros::shutdown();

    return 0;
};