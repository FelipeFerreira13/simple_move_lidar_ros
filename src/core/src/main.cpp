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

// Move Service
#include "base_controller/move_goal.h"
// Odometry Service
#include "odometry/pose_odom.h"
// OMS services
#include "vmxpi_ros_bringup/set_height.h"
#include "vmxpi_ros_bringup/set_gripper.h"
#include "vmxpi_ros_bringup/reset.h"


ros::ServiceClient move_goal_c;
ros::ServiceClient set_position_c;
ros::ServiceClient set_height_c;  
ros::ServiceClient reset_height_c; 
ros::ServiceClient set_gripper_c; 

// Functions prototypes
void set_position( double x, double y, double th );
void position_driver( double x, double y, double th );
void set_height( double height );
void reset_height( int direction );
void set_gripper( int angle );

enum GRIPPER { GRIPPER_OPEN = 50, GRIPPER_CLOSE = 150 };


int main(int argc, char **argv)
{
    ros::init(argc,argv,"main");
    ros::NodeHandle nh;

    ROS_INFO("main node is now started");

    move_goal_c    = nh.serviceClient<base_controller::move_goal>    ("base_controller/goal");
    set_position_c = nh.serviceClient<odometry::pose_odom>           ("odometry/set_position");
    set_height_c   = nh.serviceClient<vmxpi_ros_bringup::set_height> ("oms/set_height");
    reset_height_c = nh.serviceClient<vmxpi_ros_bringup::reset>      ("oms/reset");
    set_gripper_c  = nh.serviceClient<vmxpi_ros_bringup::set_gripper>("oms/set_gripper");

    ros::Duration(10).sleep();



    // Main Logic

    reset_height( -1 );

    set_height( 30 );

    set_gripper( GRIPPER_OPEN );

    set_position( 0.3, 0.3, 90 );  

    position_driver( 0.3, 1.0, 90 );

    set_height( 20 );

    set_gripper( GRIPPER_CLOSE );


    ros::spin();
    ros::shutdown();

    return 0;
};

void set_position( double x, double y, double th ){
    odometry::pose_odom pose;

    pose.request.x = x;
    pose.request.y = y;
    pose.request.th = th;

    set_position_c.call( pose );
}

void position_driver( double x, double y, double th ){
    base_controller::move_goal goal;

    goal.request.x = x;
    goal.request.y = y;
    goal.request.th = th;

    move_goal_c.call( goal );
}

void set_height( double height ){
    vmxpi_ros_bringup::set_height position;

    position.request.height = height;

    set_height_c.call(position);
}

void reset_height( int direction ){
    vmxpi_ros_bringup::reset reset;

    reset.request.direction = direction;

    reset_height_c.call( reset );
}

void set_gripper( int angle ){
    vmxpi_ros_bringup::set_gripper gripper;

    gripper.request.angle = angle;

    set_gripper_c.call( gripper );
}