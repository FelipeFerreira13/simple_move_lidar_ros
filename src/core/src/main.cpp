/************************************
 *  Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * Release date: 27/06/2024
 * 
 * Modified by: Felipe Ferreira
 * Last modification date: 02/10/2024
 * New version: 1.0.1.0

*************************************/

#include "main.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"main");
    ros::NodeHandle nh;

    ROS_INFO("main node is now started");

    move_goal_c    = nh.serviceClient<base_controller::move_goal>    ("base_controller/goal" );
    set_position_c = nh.serviceClient<odometry::pose_odom>           ("odometry/set_position");
    set_height_c   = nh.serviceClient<vmxpi_ros_bringup::set_height> ("oms/set_height"       );
    reset_height_c = nh.serviceClient<vmxpi_ros_bringup::reset>      ("oms/reset"            );
    set_gripper_c  = nh.serviceClient<vmxpi_ros_bringup::set_gripper>("oms/set_gripper"      );

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
