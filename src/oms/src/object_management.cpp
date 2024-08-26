/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * Release date: 16/08/2024
 * 
 * Modified by: Felipe Ferreira
 * Last modification date: 
 * New version:

*************************************/

#include "object_management.h"

int main(int argc, char **argv)
{
    system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
    ros::init(argc, argv, "oms_node");

    ros::NodeHandle nh; //Internal reference to the ROS node that the program will use to interact with the ROS system

    ros::ServiceServer service_driver  = nh.advertiseService("oms/set_height",   oms_driver);
    ros::ServiceServer service_reset   = nh.advertiseService("oms/reset",         oms_reset);
    ros::ServiceServer service_gripper = nh.advertiseService("oms/set_gripper", set_gripper);

    setAngle = nh.serviceClient<vmxpi_ros::Float>("channel/12/servo/set_angle");

    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, cmd_vel_callback);

    ros::Subscriber enc_sub = nh.subscribe("channel/3/encoder/count", 1, encCallback);

    ros::Subscriber limit_high_sub = nh.subscribe("/robot/digital_in/limit_switch_high/state", 1, limit_high_Callback);
    ros::Subscriber limit_low_sub  = nh.subscribe("/robot/digital_in/limit_switch_low/state", 1, limit_low_Callback );

    ros::Subscriber stop_sub  = nh.subscribe("channel/10/digital_in/state", 1, stop_Callback );

    set_m_pwm      = nh.advertise<std_msgs::Float32>("motor/3/set_motor_pwm", 1);

    get_height_pub = nh.advertise<std_msgs::Float32>("oms/height", 1);

    ros::spin();
    return 0;
}


bool oms_driver( vmxpi_ros_bringup::set_height::Request &req, vmxpi_ros_bringup::set_height::Response &res ){

    simpleControl elevatorControl;
    elevatorControl.max_motor_speed = 15;

    float desired_speed = 0;

    double desired_height = req.height;

    if ( desired_height <= high_height && desired_height >= low_height ){
        
        ros::Rate loop_rate(5);
        do{

            if( !limit_high_state ){ height = high_height; }
            if( !limit_low_state  ){ height = low_height; }

            float elev_diff  = desired_height - height;

            float max_speed = 7.5;          // [cm/s]

            desired_speed = (elev_diff / 5.0) * max_speed;
            desired_speed = std::max( std::min( desired_speed, max_speed ), -1 * max_speed );
            if( abs(elev_diff) < tolerance ){ desired_speed = 0; }

            float elevatorPWM  = elevatorControl.motorControl(desired_speed, elevatorVelocity, delta_time);

            elevatorMotor(elevatorPWM);

            ros::spinOnce();

            loop_rate.sleep();

        }while( desired_speed != 0 );

    }else{
        printf( "Desired Position out of the range %f to %f", low_height, high_height );
    }

    elevatorMotor(0.0);
    
}

// -1 to send it down and 1 to send it up
bool oms_reset( vmxpi_ros_bringup::reset::Request &req, vmxpi_ros_bringup::reset::Response &res ){
    
    ros::Rate loop_rate(5);

    if( req.direction == 1 ){
        elevatorMotor( 0.35 );
        while( limit_high_state ){ 
            ros::spinOnce();
            loop_rate.sleep(); 
        }
        height = high_height;
        ROS_INFO("new height is %f", height);
        elevatorMotor( 0 );
        return true;
    }else if (req.direction == -1){
        elevatorMotor( -0.35 );
        while( limit_low_state ){ 
            ros::spinOnce();
            loop_rate.sleep();
        }
        height = low_height;
        ROS_INFO("new height is %f", height);
        elevatorMotor( 0 );
        return true;
    }else{
        return false;
    }
}

bool set_gripper( vmxpi_ros_bringup::set_gripper::Request &req, vmxpi_ros_bringup::set_gripper::Response &res ){
    vmxpi_ros::Float msg;
    msg.request.data = req.angle;
    setAngle.call( msg );

    return true;
}


float simpleControl::motorControl(float desiredSpeed, float currentSpeed, float delta_time){

    float Desired_PWM = (desiredSpeed / max_motor_speed); // Proportional PWM regarding MAX speed of the motor [PWM]

    float error = desiredSpeed - currentSpeed;  // [cm/s]

    //Defines a proportional increment to the PWM
    double incrementPWM = (error / max_motor_speed);
    incrementPWM = incrementPWM * delta_time;

    if( error == 0 ){ correction = 0;}
    else{ correction  = std::max( std::min( correction + incrementPWM, 0.4 ), -0.4 ); }

    Desired_PWM = std::max( std::min( Desired_PWM + correction, 1.0 ),  -1.0 );

    if (desiredSpeed == 0){ Desired_PWM = 0; }

    return Desired_PWM;
}

void elevatorMotor( double pwm ){
    std_msgs::Float32 msg;
    msg.data = pwm;
    set_m_pwm.publish(msg);
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel){
    float cmd_vel_z = cmd_vel->linear.z;

    if ( cmd_vel_z != prev_cmd_z ){
        float pwm = (cmd_vel_z * 100) / 15;
        elevatorMotor( pwm );

        prev_cmd_z = cmd_vel_z;
    }
}
