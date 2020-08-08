# Control and Navigation of TurtleBot3 ground robots using MATLAB's ROS and Navigation Toolbox.

This repository contains MATLAB and C++ codes along with Gazebo files needed to simulate the motion of a robot in a environment.

## TurtleBot3

The ground robot used in this project is the [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/). Some of the models of the robot are as shown below:

<img src="images/TurtleBot3_burger.jpg" width="550"/>

<img src="images/TurtleBot3_waffle.jpg" width="550"/>

## Gazebo

[Gazebo](http://gazebosim.org/) is a open-source 3D robotics simulator using which one can simulate a environment as close to the real world as possible. Some of the environment used in this project are as shown below:

<img src="images/TB3_Environment.jpg" width="550"/>

<img src="images/house.jpg" width="550"/>

<img src="images/Cafe.jpg" width="550"/>

## ROS Toolbox

* Provides an interface connecting MATLAB and Simulink with ROS and ROS2, thereby enabling us to create a network of ROS nodes.
* Create, send, and receive messages, topics, and network information.
* Create custom ROS messages.

## Navigation Toolbox

* Provides algorithms and analysis tools for designing the motion planning and navigation systems.
* Can be used to calibrate and simulate IMU, GPS, and range sensors.
* To implement SLAM algorithms with lidar scans.
* Used to create occupancy maps of the environment.

## Simple Navigation

* Subscribe to the odometry data.
* Publish the velocity.
* Set the goal point.
* Find the current orientation (yaw) and calculate the difference in angle between the current position and the goal position.
* Robot will first rotate with a given angular velocity and then travel with a given linear velocity.

## LiDAR Scanner

The TurtleBot3 comes with a 360 Laser Distance Sensor LDS-01 and some of its features are as follows:

* Distance range: 0.12 - 3.5 m
* Sampling rate: 1.8 kHz
* Scan rate: 5 Hz

## Occupancy Map

* Occupancy grids are used to represent a robot workspace as a discrete grid.

* Information about the environment can be collected from the sensors in real-time or can be loaded from prior knowledge.

* The *binaryOccupancyMap* creates a 2D occupancy map object, which can be used to represent and visualize a robot workspace.

* The integration of sensor data and position estimates create a spatial representation of the approximate locations of the obstacles.


<img src="images/TB3_Environment.jpg" width="550"/> <img src="images/BOM_Environment.jpg" width="380"/> 
