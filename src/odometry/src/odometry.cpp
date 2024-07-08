/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * Release date: 27/06/2024
 * 
 * Modified by: Felipe Ferreira
 * Last modification date: 08/07/2024
 * New version: 1.0.0.1

*************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>

#include <iostream>

static double PI = 3.14159265;

static double x = 0.0;
static double y = 0.0;
static double th = 0.0;

//Sensor Measurement
static double LidarX = 0;
static double LidarY = 0;
static double LidarTheta = 0;

static double xDiff = 0;
static double yDiff = 0;
static double thDiff = 0;

static double navxAngle = 0;
static double navxDiff = 0;

static double lidar_angle = -90;

void setPosition( double x, double y, double th);
double Quotient_Remainder( double x, double y );
double ToDegrees(double x);
double ToRadian(double x);
double AngleLogic( double x, double ref );
void rotate( double x, double y, double phi, double *v);


void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose){
  LidarX = pose->x;
  LidarY = pose->y;
  LidarTheta = pose->theta;
}

void angleCallback(const std_msgs::Float32::ConstPtr& msg){navxAngle = msg->data * -1;}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry");

  ros::NodeHandle n;
  
  ros::Subscriber laser_pose = n.subscribe("pose2D", 1, poseCallback);
  ros::Subscriber angle_sub  = n.subscribe("navx/angle", 1, angleCallback);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 5);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 1);
  
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::Quaternion odom_quat;
  sensor_msgs::Imu imu_data;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  double last_x = 0;
  double last_y = 0;
  double last_th = 0;

  ros::Duration(5).sleep();

  setPosition(0.0, 0.0, 90.0); //Set initial Position

  ros::Rate r(10);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();

    double v[2] = {0, 0};
    rotate(LidarX, LidarY, thDiff, v);
    x = v[0] + xDiff;
    y = v[1] + yDiff;

    th = Quotient_Remainder((navxAngle + navxDiff), 360);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf::createQuaternionMsgFromYaw((PI/180) * th);

    imu_data.header.stamp = current_time;
    imu_data.header.frame_id = "imu_link";
    
    imu_data.orientation = odom_quat;
    imu_pub.publish(imu_data);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = (x - last_x) / dt;
    odom.twist.twist.linear.y = (y - last_y) / dt;
    double th_velocity = (PI/180) * AngleLogic(Quotient_Remainder( ( th - last_th ), 360 ), 360) / dt;
    odom.twist.twist.angular.z = th_velocity;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    last_x = x;
    last_y = y;
    last_th = th;

    r.sleep();
  }
}

//Set a robot position (x[m], y[m] and theta[°])
void setPosition( double set_x, double set_y, double set_th){

  thDiff = ((PI/180) * (set_th + lidar_angle) ) - LidarTheta;
  navxDiff = (set_th - navxAngle);

  double v[2] = {0, 0};
  rotate(LidarX, LidarY, thDiff, v);
  xDiff = set_x - v[0];
  yDiff = set_y - v[1];

}

double Quotient_Remainder( double x, double y ){
  double Quotient = floor( x / y );
  double Remainder = x - (y * Quotient);

  return Remainder;
}

double ToDegrees(double x){
  return ( (x / M_PI) * 180);
}

double ToRadian(double x){
  return ( (x * M_PI) / 180);
}

double AngleLogic( double x, double ref ){
  if ( x > ref/2 ) { return (x - ref); }
  else if ( x < (-1 * ref/2) ) { return (x + ref); }
  else { return x; }
}

void rotate( double x, double y, double phi, double *v){
  v[0] = (cos(phi) * x) + (sin(phi) * y * -1);
  v[1] = (sin(phi) * x) + (cos(phi) * y);
}