# simple_move_lidar_ros

This Mobile Robotics project aimed at providing straightforward tools for controlling mobile robots and interfacing with peripherals. It offers simple solutions for controlling robot movement and facilitates easy integration with various hardware components such as sensors and actuators. With clear documentation and a modular architecture, developers can quickly implement these tools into their projects.

## BASE_CONTROLLER
Defines the controls parameters and outputs for reaching desired goals.

## MAIN
Implements the main logic of the project, defining the move goals, sensors referencing, logic flow, etc.

## ODOMETRY
Defines the frame transform between base_footprint and odom. It can use either the encoder or the lidar as reference for odometry.

## ROBOT
Set the robot definitions for the frames transform and the simulation parameters, such as, size, transforms, mechanical aspect, etc.

## VMXPI_ROS
Implements the hardware interface. ( For this project it was used a VMX-pi )
