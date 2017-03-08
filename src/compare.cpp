// ROS and related stuff:
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
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
  ros::Subscriber imu_data_sub;
  ros::Subscriber imu_odometry_sub;
  ros::Subscriber visual_odometry_sub;
  ros::Subscriber jackal_odometry_sub;

public:
  // x, y, theta for each of the methods:
  geometry_msgs::Pose2D imu_data_pose;
  geometry_msgs::Pose2D imu_odo_pose;
  geometry_msgs::Pose2D visual_pose;
  geometry_msgs::Pose2D jackal_pose;

  // twists from each source:
  geometry_msgs::Twist imu_data_twist;
  geometry_msgs::Twist imu_odo_twist;
  geometry_msgs::Twist visual_twist;
  geometry_msgs::Twist jackal_twist;

  // twists at time t-1 (for deriv calcs):
  geometry_msgs::Twist imu_data_twist_prev;
  geometry_msgs::Twist imu_odo_twist_prev;
  geometry_msgs::Twist visual_twist_prev;
  geometry_msgs::Twist jackal_twist_prev;

  // use geometry_msgs::Twist containers to store relevant accels:
  geometry_msgs::Twist imu_data_accel;
  geometry_msgs::Twist imu_odo_accel;
  geometry_msgs::Twist visual_accel;
  geometry_msgs::Twist jackal_accel;

  // time variables for calculating accels:
  double t_prev_imu_data;
  double t_prev_imu_odo;
  double t_prev_visual;
  double t_prev_jackal;


public:
  DataGrab(ros::NodeHandle n) // CONSTRUCTOR
  {
    // set up nodehandle & subscriptions to different topics:
    nh = n;
    imu_data_sub = nh.subscribe("/imu/data", 1, &DataGrab::imu_data_callback, this);
    imu_odometry_sub = nh.subscribe("/odometry/imu", 1, &DataGrab::imu_odo_callback, this);
    visual_odometry_sub = nh.subscribe("/odometry/visual", 1, &DataGrab::visual_callback, this);
    jackal_odometry_sub = nh.subscribe("/odometry/filtered", 1, &DataGrab::jackal_callback, this);

    // initialize times:
    t_prev_imu_odo = ros::Time::now().toSec();
    t_prev_imu_data = ros::Time::now().toSec();
    t_prev_visual = ros::Time::now().toSec();
    t_prev_jackal = ros::Time::now().toSec();

  } // END OF CONSTRUCTOR


private:
  void imu_data_callback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    double t_curr = ros::Time::now().toSec();
    double dt = t_curr - t_prev_imu_data;

    double R, P, Y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q); // transform from geometry_msgs::quaternion to tf::quaternion
    tf::Matrix3x3 m(q); // create 3x3 matrix from quaternion
    m.getRPY(R, P, Y); // extract RPY

    imu_data_pose.x = 0; // sensor_msgs::IMU data doesn't contain xyz info
    imu_data_pose.y = 0; // sensor_msgs::IMU data doesn't contain xyz info
    imu_data_pose.theta = Y;

    // calculate accel & twist data in best order to minimize function / network calls
    imu_data_accel.linear.x = msg->linear_acceleration.x;
    imu_data_accel.linear.y = msg->linear_acceleration.y;
    imu_data_accel.linear.z = msg->linear_acceleration.z;
    imu_data_twist.angular.x = msg->angular_velocity.x;
    imu_data_twist.angular.y = msg->angular_velocity.y;
    imu_data_twist.angular.z = msg->angular_velocity.z;

    // then derive remainder from stored data
    // imu_data_twist.linear.x = imu_data_accel.linear.x * dt;
    // imu_data_twist.linear.y = imu_data_accel.linear.y * dt;
    // imu_data_twist.linear.z = imu_data_accel.linear.z * dt;
    imu_data_accel.angular.x = (imu_data_twist.angular.x - imu_data_twist_prev.angular.x)/ dt;
    imu_data_accel.angular.y = (imu_data_twist.angular.y - imu_data_twist_prev.angular.y) / dt;
    imu_data_accel.angular.z = (imu_data_twist.angular.z - imu_data_twist_prev.angular.z) / dt;

    imu_data_twist_prev = imu_data_twist; // for next round

    t_prev_imu_data = t_curr; // update time for next call

  } // END OF imu_data_callback() FUNCTION


  void imu_odo_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    double t_curr = ros::Time::now().toSec();
    double dt = t_curr - t_prev_imu_odo;

    // extract RPY from incoming message
    double R, P, Y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q); // transform from geometry_msgs::quaternion to tf::quaternion
    tf::Matrix3x3 m(q); // create 3x3 matrix from quaternion
    m.getRPY(R, P, Y); // extract RPY

    // and store values in Pose2D
    imu_odo_pose.x = msg->pose.pose.position.x;
    imu_odo_pose.y = msg->pose.pose.position.y;
    imu_odo_pose.theta = Y;

    // extract twist data
    imu_odo_twist.linear.x = msg->twist.twist.linear.x;
    imu_odo_twist.linear.y = msg->twist.twist.linear.y;
    imu_odo_twist.linear.z = msg->twist.twist.linear.z;
    imu_odo_twist.angular.x = msg->twist.twist.angular.x;
    imu_odo_twist.angular.y = msg->twist.twist.angular.y;
    imu_odo_twist.angular.z = msg->twist.twist.angular.z;

    // calculate accel data
    imu_odo_accel.linear.x = (imu_odo_twist.linear.x - imu_odo_twist_prev.linear.x) / dt;
    imu_odo_accel.linear.y = (imu_odo_twist.linear.y - imu_odo_twist_prev.linear.y) / dt;
    imu_odo_accel.linear.z = (imu_odo_twist.linear.z - imu_odo_twist_prev.linear.z) / dt;
    imu_odo_accel.angular.x = (imu_odo_twist.angular.x - imu_odo_twist_prev.angular.x) / dt;
    imu_odo_accel.angular.y = (imu_odo_twist.angular.y - imu_odo_twist_prev.angular.y) / dt;
    imu_odo_accel.angular.z = (imu_odo_twist.angular.z - imu_odo_twist_prev.angular.z) / dt;

    imu_odo_twist_prev = imu_odo_twist; // for next round

    t_prev_imu_odo = t_curr; // update time for next call

  } // END OF imu_odo_callback() FUNCTION


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
    visual_accel.linear.x = (visual_twist.linear.x - visual_twist_prev.linear.x) / dt;
    visual_accel.linear.y = (visual_twist.linear.y - visual_twist_prev.linear.y) / dt;
    visual_accel.linear.z = (visual_twist.linear.z - visual_twist_prev.linear.z) / dt;
    visual_accel.angular.x = (visual_twist.angular.x - visual_twist_prev.angular.x) / dt;
    visual_accel.angular.y = (visual_twist.angular.y - visual_twist_prev.angular.y) / dt;
    visual_accel.angular.z = (visual_twist.angular.z - visual_twist_prev.angular.z) / dt;

    // cout << "visual_twist: curr, prev, dt = " << setw(12) << visual_twist.angular.z  << setw(12) << visual_twist_prev.angular.z  << setw(12) << dt << endl;

    visual_twist_prev = visual_twist; // for next round

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
    jackal_accel.linear.x = (jackal_twist.linear.x - jackal_twist_prev.linear.x) / dt;
    jackal_accel.linear.y = (jackal_twist.linear.y - jackal_twist_prev.linear.y) / dt;
    jackal_accel.linear.z = (jackal_twist.linear.z - jackal_twist_prev.linear.z) / dt;
    jackal_accel.angular.x = (jackal_twist.angular.x - jackal_twist_prev.angular.x) / dt;
    jackal_accel.angular.y = (jackal_twist.angular.y - jackal_twist_prev.angular.y) / dt;
    jackal_accel.angular.z = (jackal_twist.angular.z - jackal_twist_prev.angular.z) / dt;

    jackal_twist_prev = jackal_twist; // for next round

    t_prev_jackal = t_curr; // update time for next call

  } // END OF jackal_callback() FUNCTION

}; // END OF CLASS DataGrab


int main(int argc, char** argv)
{
  // set up node and instantiate callback class:
  ros::init(argc, argv, "analysis_node");
  ros::NodeHandle nh;
  ros::Publisher imu_pub_data = nh.advertise<geometry_msgs::Twist>("/accel/imu_data", 1);
  ros::Publisher imu_pub_odo = nh.advertise<geometry_msgs::Twist>("/accel/imu_odo", 1);
  ros::Publisher visual_pub = nh.advertise<geometry_msgs::Twist>("/accel/visual", 1);
  ros::Publisher jackal_pub = nh.advertise<geometry_msgs::Twist>("/accel/jackal", 1);

  // initialize data handler class:
  DataGrab c(nh);

  // loop to publish and print relevant data:
  int j = 0; // counter
  while(ros::ok())
  {
    // if(j == 0)
    // { // print out heading info:
    //   cout << endl;
    //   cout << setw(3 * WIDTH) << "JACKAL:" << setw(3 * WIDTH) << "VISUAL:" << setw(3 * WIDTH) << "IMU:" << endl;
    //   for(int i = 0; i < 3; ++i)
    //   { // 3 topics, 3 sets of columns
    //     cout << setw(WIDTH) << "x:" << setw(WIDTH) << "y:" << setw(WIDTH) << "theta:";
    //   }
    //   cout << endl;
    // }

    // print out pose info:
    // cout << setw(WIDTH) << c.jackal_pose.x << setw(WIDTH) << c.jackal_pose.y << setw(WIDTH) << c.jackal_pose.theta;
    // cout << setw(WIDTH) << c.visual_pose.x << setw(WIDTH) << c.visual_pose.y << setw(WIDTH) << c.visual_pose.theta;
    // cout << setw(WIDTH) << c.imu_odo_pose.x << setw(WIDTH) << c.imu_odo_pose.y << setw(WIDTH) << c.imu_odo_pose.theta << endl;
    // cout << setw(WIDTH) << c.jackal_accel.linear.x << setw(WIDTH) << c.jackal_accel.angular.z;
    // cout << setw(WIDTH) << c.visual_accel.linear.x << setw(WIDTH) << c.visual_accel.angular.z;
    // cout << setw(WIDTH) << c.imu_odo_accel.linear.x << setw(WIDTH) << c.imu_odo_accel.angular.z << endl;
    ros::spinOnce(); // required to invoke callbacks

    // send data for plotting:
    imu_pub_data.publish(c.imu_data_accel);
    imu_pub_odo.publish(c.imu_odo_accel);
    visual_pub.publish(c.visual_accel);
    jackal_pub.publish(c.jackal_accel);

    ros::Duration(0.1).sleep(); // 10 Hz

    ++j %= 30; // reprint column heades every 30 lines
  }

  return 0;
}
