// ros and related stuff:
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
// #include <tf/tf.h>

// core c++ stuff:
#include <iostream>
#include <vector>
#include <cmath> // sqrt
#include <iomanip> // std::setw
#include <algorithm>


using namespace std;


class DataGrab
{
private:
  // set up ROS stuff:
  ros::NodeHandle nh;
  ros::Subscriber imu_odometry_sub;
  ros::Subscriber visual_odometry_sub;
  ros::Subscriber jackal_odometry_sub;

public:
  // x, y, theta for each of the methods:
  geometry_msgs::Pose2D imu_pose;
  geometry_msgs::Pose2D visual_pose;
  geometry_msgs::Pose2D jackal_pose;


public:
  DataGrab()
  {
    // set up subscriptions to 3 different topics:
    imu_odometry_sub = nh.subscribe("/odometry/imu", 1, &DataGrab::imu_callback, this);
    visual_odometry_sub = nh.subscribe("/odometry/visual", 1, &DataGrab::visual_callback, this);
    jackal_odometry_sub = nh.subscribe("/odometry/filtered", 1, &DataGrab::jackal_callback, this);

  } // END OF CONSTRUCTOR

private:
  void imu_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // extract RPY from incoming message
    double R, P, Y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q); // transform from geometry_msgs::quaternion to tf::quaternion
    tf::Matrix3x3 m(q); // create 3x3 matrix from quaternion
    m.getRPY(R, P, Y); // extract RPY

    // and store values in Pose2D
    imu_pose.x = msg->pose.pose.position.x;
    imu_pose.y = msg->pose.pose.position.y;
    imu_pose.theta = Y;

  } // END OF imu_callback() FUNCTION


  void visual_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // extract RPY from incoming message
    double R, P, Y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q); // transform from geometry_msgs::quaternion to tf::quaternion
    tf::Matrix3x3 m(q); // create 3x3 matrix from quaternion
    m.getRPY(R, P, Y); // extract RPY

    // and store values in Pose2D
    visual_pose.x = msg->pose.pose.position.x;
    visual_pose.y = msg->pose.pose.position.y;
    visual_pose.theta = Y;

  } // END OF visual_callback() FUNCTION


  void jackal_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // extract RPY from incoming message
    double R, P, Y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q); // transform from geometry_msgs::quaternion to tf::quaternion
    tf::Matrix3x3 m(q); // create 3x3 matrix from quaternion
    m.getRPY(R, P, Y); // extract RPY

    // and store values in Pose2D
    jackal_pose.x = msg->pose.pose.position.x;
    jackal_pose.y = msg->pose.pose.position.y;
    jackal_pose.theta = Y;

  } // END OF jackal_callback() FUNCTION

}; // END OF CLASS DataGrab


int main(int argc, char** argv)
{
  // set up node and instantiate callback class
  ros::init(argc, argv, "analysis_node");
  DataGrab c;

  int j = 0; // counter
  int width = 12; // column spacing
  while(ros::ok())
  {
    if(j == 0)
    { // print out heading info:
      cout << endl;
      cout << setw(3 * width) << "JACKAL:" << setw(3 * width) << "VISUAL:" << setw(3 * width) << "IMU:" << endl;
      for(int i = 0; i < 3; ++i)
      { // 3 topics, 3 sets of columns
        cout << setw(width) << "x:" << setw(width) << "y:" << setw(width) << "theta:";
      }
      cout << endl;
    }

    // print out pose info:
    cout << setw(width) << c.jackal_pose.x << setw(width) << c.jackal_pose.y << setw(width) << c.jackal_pose.theta;
    cout << setw(width) << c.visual_pose.x << setw(width) << c.visual_pose.y << setw(width) << c.visual_pose.theta;
    cout << setw(width) << c.imu_pose.x << setw(width) << c.imu_pose.y << setw(width) << c.imu_pose.theta << endl;
    ros::spinOnce(); // required to invoke callbacks
    ros::Duration(0.1).sleep();

    ++j %= 30; // return every 25 data points
  }

  return 0;
}
