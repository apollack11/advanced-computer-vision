Using Optical Flow to Estimate Robot State
==============
#### EECS432 Advanced Computer Vision Final Project
##### Adam Pollack & Nate Kaiser
##### *Northwestern University, Winter 2017*

![][1]

## Overview
This package provides basic state (*i.e.* linear and angular position, velocity, acceleration) estimation functionality using optical flow data from a camera rigidly mounted to the chassis of the robot. The camera data is fused with IMU data using an [Extended Kalman Filter][2] to improve accuracy. Keep reading for specifics.


## Installation
Make sure you have [ROS indigo][3] and [OpenCV 2.4][4] installed and working on the host computer and robot. Then, to install this package:

1. Clone [this repo][5] into a workspace and build<sup>&dagger;</sup>
2. SSH into Jackal and repeat the process

<sup>&dagger;</sup>*I highly recommend using [catkin_tools][11], which has a modernized version of the catkin_make command and other useful command line tools for working with ROS/catkin packages*


## Instructions For Use
Navigate to your workspace and source your setup file: `source devel/setup.bash`. Then run `roslaunch eecs432_project <launch file name>` for any of the launch files below.

#### Launch Files:
**project.launch**
Launches both `ekf_robot_localization.launch` and `ekf_robot_localization_imu.launch`. Publishes `nav_msgs/Odometry` messages to `/odometry/visual` and `/odometry/imu` topics.

**ekf_robot_localization.launch**
Launches nodes necessary to extract motion from a camera feed (`flow.cpp`), converts that output to usable data (`twist_data.py`), and fuses visual and IMU data to create visual-inertial odometry (`robot_localization`). Publishes a `nav_msgs/Odometry` message to `/odometry/visual`.

**ekf_robot_localization_imu.launch**
Launches only the `robot_localization` node to convert IMU data to odometry data to be consistent in format with the other odometry outputs (position and velocity in x, y, z, roll, pitch, yaw). Publishes a `nav_msgs/Odometry` message on topic `/odometry/imu`.

#### Source Code:
**flow.cpp**
Extracts features, calculates optical flow, and performs perspective transform on a live camera feed. Publishes a `geometry_msgs/Twist` message to `/optical_flow/twist` which contains twist data in the form of velocity in x, y, z, roll, pitch, and yaw.

**twist_data.py**
Converts output of `flow.cpp` from a `geometry_msgs/Twist` message to a `nav_msgs/Odometry` message which contains measurement covariances in addition to the original Twist data. This message is published to the `/optical_flow` topic.

**robot_localization**
ROS package used to perform fusion of multiple streams of sensor data. Converts the output to position and velocity in x, y, z, roll, pitch, and yaw. Publishes to `/odometry/filtered` by default. The documentation can be found [here][6].

![][7]

<!-- File Locations -->
[1]: https://github.com/apollack11/advanced-computer-vision/blob/master/media/optical_flow_points_room.gif
[2]: https://en.wikipedia.org/wiki/Extended_Kalman_filter
[3]: http://wiki.ros.org/indigo
[4]: http://docs.opencv.org/2.4.13.2/
[5]: https://github.com/apollack11/advanced-computer-vision
[6]: http://wiki.ros.org/robot_localization?distro=indigo
[7]: https://github.com/apollack11/advanced-computer-vision/blob/master/media/jackal_rotating.gif
