#!/usr/bin/env python
import rospy
import tf
import sys
import csv
from nav_msgs.msg import Odometry

imu_odom = []
wheel_odom = []
visual_odom = []

class data_export:
    def __init__(self):
        self.imu_odom_sub = rospy.Subscriber('/odometry/imu', Odometry, self.imu_odom_callback)
        self.wheel_odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.wheel_odom_callback)
        self.visual_odom_sub = rospy.Subscriber('/odometry/visual', Odometry, self.visual_odom_callback)

    def imu_odom_callback(self,data):
        x = data.pose.pose.position.x
        x_vel = data.twist.twist.linear.x
        yaw = data.pose.pose.orientation.x
        yaw_vel = data.twist.twist.angular.z
        imu_odom.append([x,x_vel,yaw,yaw_vel])

    def wheel_odom_callback(self,data):
        x = data.pose.pose.position.x
        x_vel = data.twist.twist.linear.x
        yaw = data.pose.pose.orientation.x
        yaw_vel = data.twist.twist.angular.z
        wheel_odom.append([x,x_vel,yaw,yaw_vel])

    def visual_odom_callback(self,data):
        x = data.pose.pose.position.x
        x_vel = data.twist.twist.linear.x
        yaw = data.pose.pose.orientation.x
        yaw_vel = data.twist.twist.angular.z
        visual_odom.append([x,x_vel,yaw,yaw_vel])

def getch():   # define key detector
    import tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def write_csv(data,filename):
    f = open(filename, 'w+')

    writer = csv.writer(f)
    writer.writerow(['x','x_vel','yaw','yaw_vel'])
    for values in data:
        writer.writerow(values)

    f.close()

def main():
    rospy.init_node('data_export', anonymous=True)
    de = data_export()

    while not rospy.is_shutdown():
        key = getch()
        if key == 's':
            write_csv(imu_odom,"imu_odom.csv")
            write_csv(wheel_odom,"wheel_odom.csv")
            write_csv(visual_odom,"visual_odom.csv")
            print "Data Saved"
        if key == 'q':
            quit()

if __name__ == '__main__':
    main()
