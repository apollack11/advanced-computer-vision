# advanced-computer-vision
Final project for EECS432 Advanced Computer Vision

## Contents

### Nodes  
**flow.cpp**  
Extracts features, calculates optical flow, and performs perspective transform on a live camera feed. Publishes a Twist message to `/optical_flow/twist` which contains twist data in the form of velocity in x, y, z, roll, pitch, and yaw

**twist_data.py**  
Converts output of `flow.cpp` from a Twist message to an Odometry message which contains measurement covariances in addition to the original Twist data. This message is published to `/optical_flow`  

**robot_localization**  
ROS package used to perform sensor fusion with multiple streams of data and convert the output to position and velocity in x, y, z, roll, pitch, yaw. The default publisher publishes to `/odometry/filtered`

### Launch Files  
**project.launch**  
Launches both the `ekf_robot_localization.launch` launch and the `ekf_robot_localization_imu.launch` launch file. Publishes Odometry messages to `/odometry/visual` and `/odometry/imu`

**ekf_robot_localization.launch**  
Launches nodes necessary to extract motion from a camera feed (`flow.cpp`), convert that output to useable data (`twist_data.py`), and fuse visual data with IMU data to create visual-inertial odometry (`robot_localization`). Publishes Odometry message to `/odometry/visual`

**ekf_robot_localization_imu.launch**  
Launches only the `robot_localization` node to convert the IMU data to odometry data which is consistent in format to the other odometry outputs (position and velocity in x, y, z, roll, pitch, yaw). Publishes Odometry message `/odometry/imu`
