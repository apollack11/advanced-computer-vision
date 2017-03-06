// ROS and related stuff:
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

// core c++ stuff:
#include <iostream>
#include <vector>
#include <cmath> // sqrt
#include <iomanip> // std::setw
#include <algorithm>

#define WIDTH 12 // column spacing

using namespace std;


class DataGrab
{
private:
  // set up ROS stuff:
  ros::NodeHandle nh;
  ros::Subscriber imu_odometry_sub;
  ros::Subscriber imu_data_sub;
  ros::Subscriber visual_odometry_sub;
  ros::Subscriber jackal_odometry_sub;

public:
  // x, y, theta for each of the methods:
  geometry_msgs::Pose2D imu_pose;
  geometry_msgs::Pose2D visual_pose;
  geometry_msgs::Pose2D jackal_pose;

  // twists from each source:
  geometry_msgs::Twist imu_twist;
  geometry_msgs::Twist visual_twist;
  geometry_msgs::Twist jackal_twist;

  // also using geometry_msgs::Twist containers to store relevant accels:
  geometry_msgs::Twist imu_accel;
  geometry_msgs::Twist visual_accel;
  geometry_msgs::Twist jackal_accel;

  // time variables for calculating accels:
  double t_prev_imu;
  double t_prev_visual;
  double t_prev_jackal;


public:
  DataGrab(ros::NodeHandle n) // CONSTRUCTOR
  {
    // set up nodehandle & subscriptions to 3 different topics:
    nh = n;
    imu_odometry_sub = nh.subscribe("/odometry/imu", 1, &DataGrab::imu_callback, this);
    visual_odometry_sub = nh.subscribe("/odometry/visual", 1, &DataGrab::visual_callback, this);
    jackal_odometry_sub = nh.subscribe("/odometry/filtered", 1, &DataGrab::jackal_callback, this);

    // initialize times:
    t_prev_imu = ros::Time::now().toSec();
    t_prev_visual = ros::Time::now().toSec();
    t_prev_jackal = ros::Time::now().toSec();

  } // END OF CONSTRUCTOR


private:
  void imu_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    double t_curr = ros::Time::now().toSec();
    double dt = t_curr - t_prev_imu;

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

    // extract twist data
    imu_twist.linear.x = msg->twist.twist.linear.x;
    imu_twist.linear.y = msg->twist.twist.linear.y;
    imu_twist.linear.z = msg->twist.twist.linear.z;
    imu_twist.angular.x = msg->twist.twist.angular.x;
    imu_twist.angular.y = msg->twist.twist.angular.y;
    imu_twist.angular.z = msg->twist.twist.angular.z;

    // calculate accel data
    imu_accel.linear.x = imu_twist.linear.x / dt;
    imu_accel.linear.y = imu_twist.linear.y / dt;
    imu_accel.linear.z = imu_twist.linear.z / dt;
    imu_accel.angular.x = imu_twist.angular.x / dt;
    imu_accel.angular.y = imu_twist.angular.y / dt;
    imu_accel.angular.z = imu_twist.angular.z / dt;

    t_prev_imu = t_curr; // update time for next call

  } // END OF imu_callback() FUNCTION


  void visual_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    double t_curr = ros::Time::now().toSec();
    double dt = t_curr - t_prev_visual;

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

    // extract twist data
    visual_twist.linear.x = msg->twist.twist.linear.x;
    visual_twist.linear.y = msg->twist.twist.linear.y;
    visual_twist.linear.z = msg->twist.twist.linear.z;
    visual_twist.angular.x = msg->twist.twist.angular.x;
    visual_twist.angular.y = msg->twist.twist.angular.y;
    visual_twist.angular.z = msg->twist.twist.angular.z;

    // calculate accel data
    visual_accel.linear.x = visual_twist.linear.x / dt;
    visual_accel.linear.y = visual_twist.linear.y / dt;
    visual_accel.linear.z = visual_twist.linear.z / dt;
    visual_accel.angular.x = visual_twist.angular.x / dt;
    visual_accel.angular.y = visual_twist.angular.y / dt;
    visual_accel.angular.z = visual_twist.angular.z / dt;

    t_prev_visual = t_curr; // update time for next call

  } // END OF visual_callback() FUNCTION


  void jackal_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    double t_curr = ros::Time::now().toSec();
    double dt = t_curr - t_prev_jackal;

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

    // extract twist data
    jackal_twist.linear.x = msg->twist.twist.linear.x;
    jackal_twist.linear.y = msg->twist.twist.linear.y;
    jackal_twist.linear.z = msg->twist.twist.linear.z;
    jackal_twist.angular.x = msg->twist.twist.angular.x;
    jackal_twist.angular.y = msg->twist.twist.angular.y;
    jackal_twist.angular.z = msg->twist.twist.angular.z;

    // calculate accel data
    jackal_accel.linear.x = jackal_twist.linear.x / dt;
    jackal_accel.linear.y = jackal_twist.linear.y / dt;
    jackal_accel.linear.z = jackal_twist.linear.z / dt;
    jackal_accel.angular.x = jackal_twist.angular.x / dt;
    jackal_accel.angular.y = jackal_twist.angular.y / dt;
    jackal_accel.angular.z = jackal_twist.angular.z / dt;

    t_prev_jackal = t_curr; // update time for next call

  } // END OF jackal_callback() FUNCTION

}; // END OF CLASS DataGrab


int main(int argc, char** argv)
{
  // set up node and instantiate callback class:
  ros::init(argc, argv, "analysis_node");
  ros::NodeHandle nh;
  ros::Publisher imu_pub = nh.advertise<geometry_msgs::Twist>("/accel/imu", 1);
  ros::Publisher visual_pub = nh.advertise<geometry_msgs::Twist>("/accel/visual", 1);
  ros::Publisher jackal_pub = nh.advertise<geometry_msgs::Twist>("/accel/jackal", 1);

  // initialize data handler class:
  DataGrab c(nh);

  // loop to publish and print relevant data:
  int j = 0; // counter
  while(ros::ok())
  {
    if(j == 0)
    { // print out heading info:
      cout << endl;
      cout << setw(3 * WIDTH) << "JACKAL:" << setw(3 * WIDTH) << "VISUAL:" << setw(3 * WIDTH) << "IMU:" << endl;
      for(int i = 0; i < 3; ++i)
      { // 3 topics, 3 sets of columns
        cout << setw(WIDTH) << "x:" << setw(WIDTH) << "y:" << setw(WIDTH) << "theta:";
      }
      cout << endl;
    }

    // print out pose info:
    // cout << setw(WIDTH) << c.jackal_pose.x << setw(WIDTH) << c.jackal_pose.y << setw(WIDTH) << c.jackal_pose.theta;
    // cout << setw(WIDTH) << c.visual_pose.x << setw(WIDTH) << c.visual_pose.y << setw(WIDTH) << c.visual_pose.theta;
    // cout << setw(WIDTH) << c.imu_pose.x << setw(WIDTH) << c.imu_pose.y << setw(WIDTH) << c.imu_pose.theta << endl;
    cout << setw(WIDTH) << c.jackal_accel.linear.x << setw(WIDTH) << c.jackal_accel.angular.z;
    cout << setw(WIDTH) << c.visual_accel.linear.x << setw(WIDTH) << c.visual_accel.angular.z;
    cout << setw(WIDTH) << c.imu_accel.linear.x << setw(WIDTH) << c.imu_accel.angular.z << endl;
    ros::spinOnce(); // required to invoke callbacks

    // send data for plotting:
    imu_pub.publish(c.imu_accel);
    visual_pub.publish(c.visual_accel);
    jackal_pub.publish(c.jackal_accel);

    ros::Duration(0.1).sleep(); // 10 Hz

    ++j %= 30; // reprint column heades every 30 lines
  }

  return 0;
}
