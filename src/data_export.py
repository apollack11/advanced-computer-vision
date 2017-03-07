#!/usr/bin/env python
import rospy
import tf
import sys
import csv
from geometry_msgs.msg import Twist

imu_data = []
imu_odo = []
visual = []
jackal = []

class data_export:
    def __init__(self):
        self.imu_data_sub = rospy.Subscriber('/accel/imu_data', Twist, self.imu_data_callback)
        self.imu_odo_sub = rospy.Subscriber('/accel/imu_odo', Twist, self.imu_odo_callback)
        self.visual_sub = rospy.Subscriber('/accel/visual', Twist, self.visual_callback)
        self.jackal_sub = rospy.Subscriber('/accel/jackal', Twist, self.jackal_callback)

    def imu_data_callback(self,data):
        x = data.linear.x
        y = data.linear.y
        z = data.linear.z
        roll = data.angular.x
        pitch = data.angular.y
        yaw = data.angular.z
        imu_data.append([x,y,z,roll,pitch,yaw])

    def imu_odo_callback(self,data):
        x = data.linear.x
        y = data.linear.y
        z = data.linear.z
        roll = data.angular.x
        pitch = data.angular.y
        yaw = data.angular.z
        imu_odo.append([x,y,z,roll,pitch,yaw])

    def visual_callback(self,data):
        x = data.linear.x
        y = data.linear.y
        z = data.linear.z
        roll = data.angular.x
        pitch = data.angular.y
        yaw = data.angular.z
        visual.append([x,y,z,roll,pitch,yaw])

    def jackal_callback(self,data):
        x = data.linear.x
        y = data.linear.y
        z = data.linear.z
        roll = data.angular.x
        pitch = data.angular.y
        yaw = data.angular.z
        jackal.append([x,y,z,roll,pitch,yaw])

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
    writer.writerow(['x','y','z','roll','pitch','yaw'])
    for values in data:
        writer.writerow(values)

    f.close()

def main():
    rospy.init_node('data_export', anonymous=True)
    de = data_export()

    while not rospy.is_shutdown():
        key = getch()
        if key == 's':
            write_csv(imu_data,"imu_data.csv")
            write_csv(imu_odo,"imu_odo.csv")
            write_csv(visual,"visual.csv")
            write_csv(jackal,"jackal.csv")
            print "Data Saved"
        if key == 'q':
            quit()

if __name__ == '__main__':
    main()
