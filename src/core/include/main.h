

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

enum GRIPPER { GRIPPER_OPEN = 50, GRIPPER_CLOSE = 150 };


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