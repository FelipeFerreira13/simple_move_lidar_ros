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

int main(int argc, char **argv)
{
    ros::init(argc,argv,"main");
    ros::NodeHandle nh;


    ros::spin();
    ros::shutdown();

    return 0;
};