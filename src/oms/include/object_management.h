#include "ros/ros.h"
#include <unistd.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include "vmxpi_ros/Float.h"

#include "oms/set_height.h"
#include "oms/set_gripper.h"
#include "oms/reset.h"


#define ticksPerRev  1464   //Encoder pulses
#define pinionRadius 1.25   // Pinion's radius [cm]

static float height = 1000;         // OMS height [cm]

static float low_height = 10.5;     // [cm]
static float high_height = 44;      // [cm]

static float tolerance  = 0.5;      // [cm]

static double elevator_enc;         // Encoder Ticks count

static float elevatorVelocity;

static bool limit_high_state;
static bool limit_low_state;

static bool stop_button;

static float prev_cmd_z;

static double current_time = 0;
static double previous_time = 0;
inline double delta_time;

static int current_enc = elevator_enc;
static int previous_enc = elevator_enc;

ros::Publisher set_m_pwm;
ros::Publisher get_height_pub;
ros::ServiceClient setAngle;

class simpleControl{
    private:
        double correction = 0; // [PWM]

    public:   
        float max_motor_speed = 60.0; // [cm/s]

        simpleControl(){}

        float motorControl(float desiredSpeed, float currentSpeed, float delta_time);
};

bool oms_driver ( oms::set_height::Request &req, oms::set_height::Response &res );
bool set_gripper( oms::set_gripper::Request &req, oms::set_gripper::Response &res );
bool oms_reset  ( oms::reset::Request &req, oms::reset::Response &res );

void encCallback(const std_msgs::Int32::ConstPtr& msg){ 
    elevator_enc = msg->data;
    
    current_time = ros::Time::now().toSec();
    if ( previous_time == 0 ){ previous_time = current_time; }
    delta_time = current_time - previous_time; // [s]
    previous_time = current_time;

    current_enc = elevator_enc;
    float delta_enc = current_enc - previous_enc;
    previous_enc = current_enc;

    //Pinion Velocity
    elevatorVelocity  = (((2 * M_PI * pinionRadius * delta_enc) / (ticksPerRev * delta_time)));   // [cm/s]

    if ( isnan(elevatorVelocity) || isinf(elevatorVelocity) ){ elevatorVelocity  = 0; }
    
    //Elevation Displacement
    float delta_elev  = elevatorVelocity  * delta_time; // Displacement per iteration

    height = height + delta_elev;   // [cm]

    std_msgs::Float32 h;
    h.data = height;
    get_height_pub.publish( h );
}

void limit_high_Callback(const std_msgs::Bool::ConstPtr& msg){ limit_high_state = msg->data; }
void limit_low_Callback (const std_msgs::Bool::ConstPtr& msg){ limit_low_state  = msg->data; }

void stop_Callback (const std_msgs::Bool::ConstPtr& msg){ stop_button  = msg->data; }

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel);

void elevatorMotor( double pwm );

