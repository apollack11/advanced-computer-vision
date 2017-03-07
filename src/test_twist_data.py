#!/usr/bin/env python
import time
import rospy
import math
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('test_twist_pub', anonymous=True)
    pub = rospy.Publisher('accel/imu_data', Twist, queue_size=10)
    rate = rospy.Rate(60) # 60hz
    twist = Twist()
    T = 5 # amount of time for it to complete a figure 8

    start_time = time.time()

    while not rospy.is_shutdown():
        # FORMULAS FOR OUTPUTTING POSITION
        t = time.time()-start_time # time elapsed since the program started running

        x = 3*math.sin(4*math.pi*t/T)
        y = 3*math.sin(2*math.pi*t/T)
        v_x = 3*(4*math.pi/T)*math.cos(4*math.pi*t/T) # derivative of x
        v_y = 3*(2*math.pi/T)*math.cos(2*math.pi*t/T) # derivative of y

        linear_v = math.sqrt(v_x**2+v_y**2) # amount the turtle should move in the forward direction

        a_x = -3*((4*math.pi/T)**2)*math.sin(4*math.pi*t/T) # derivative of v_x
        a_y = -3*((2*math.pi/T)**2)*math.sin(2*math.pi*t/T) # derivative of v_y

        theta = math.atan(v_y/v_x)
        angular_v = 1/(1+(v_y/v_x)**2)*((a_y*v_x-a_x*v_y)/(v_x**2)) # derivative of theta

        twist.linear.x = linear_v
        twist.angular.z = angular_v
        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    main()
