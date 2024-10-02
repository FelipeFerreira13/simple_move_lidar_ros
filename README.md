# simple_move_lidar_ros

This Mobile Robotics project aimed at providing straightforward tools for controlling mobile robots and interfacing with peripherals. It offers simple solutions for controlling robot movement and facilitates easy integration with various hardware components such as sensors and actuators. With clear documentation and a modular architecture, developers can quickly implement these tools into their projects.

## CORE
Implements the main logic of the project, defining the move goals, sensors referencing, logic flow, etc.

## CAMERA
Implements the image processing solutions with its functions and services.

## ROBOT
Set the robot definitions for the frames transform and the simulation parameters, such as, size, transforms, mechanical aspect, etc.

## BASE_CONTROLLER
Defines the controls parameters and outputs for reaching desired goals.

### msgs
| **Name**            | **Request**                      | **Response** |
|---------------------|----------------------------------|--------------|
| base_controller::move_goal | float64 x, float64 y, float64 th | bool status  |

### Service Server
| **Topic**               | **msg**            | **Description**                             |
|-------------------------|--------------------|---------------------------------------------|
| `base_controller/goal` | base_controller::move_goal | Define the desired goal |

## ODOMETRY
Defines the frame transform between base_footprint and odom. It can use either the encoder or the lidar as reference for odometry.

### msgs
| **Name**            | **Request**                      | **Response** |
|---------------------|----------------------------------|--------------|
| odometry::pose_odom | float64 x, float64 y, float64 th | bool status  |

### Publisher
| **Topic**               | **msg**            | **Description**                             |
|-------------------------|--------------------|---------------------------------------------|
| `odom`                  | nav_msgs::Odometry | Publish the to odom the new odometry pose   |
| `imu/data`              | sensor_msgs::Imu   | Publish the imu angle based on the robot Th |

### Service Server
| **Topic**               | **msg**            | **Description**                             |
|-------------------------|--------------------|---------------------------------------------|
| `odometry/set_position` | odometry::pose_odom | Redifine the robot position in X, Y and Th |

## OMS
Implements the basic controls of the robot's object management system.

### msgs
| **Name**            | **Request**                      | **Response** |
|---------------------|----------------------------------|--------------|
| oms/set_height | float64 x, float64 y, float64 th | bool status  |

### Publisher
| **Topic**               | **msg**           | **Description**                      |
|-------------------------|-------------------|--------------------------------------|
| `motor/3/set_motor_pwm` | std_msgs::Float32 | Publish the desired motor pwm value  |
| `oms/height`            | std_msgs::Float32 | Publish the current elevation height |

### Service Server
| **Topic**               | **msg**            | **Description**                             |
|-------------------------|--------------------|---------------------------------------------|
| `oms/set_height` | oms::set_height | Redifine the robot position in X, Y and Th |
| `oms/reset` | oms::reset | Redifine the robot position in X, Y and Th |
| `oms/set_gripper` | oms::set_gripper | Redifine the robot position in X, Y and Th |


## VMXPI_ROS
Implements the hardware interface. ( For this project it was used a VMX-pi )
