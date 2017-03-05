#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

def main():
    rospy.init_node('optical_flow_test_data', anonymous=True)
    optical_flow_pub = rospy.Publisher('optical_flow', Odometry, queue_size=10) # what msg type??
    rate = rospy.Rate(30) # 30Hz because 30fps
    visual_odom = Odometry()

    while not rospy.is_shutdown():
        current_time = rospy.get_rostime()

        visual_odom.pose.pose.position.x = 0.0
        visual_odom.pose.pose.position.y = 0.0
        visual_odom.pose.pose.position.z = 0.0
        # visual_odom.pose.pose.orientation = [quaternion info]
        visual_odom.header.frame_id = 'front_camera_optical'
        visual_odom.header.stamp = current_time
        visual_odom.pose.covariance[0] = 0.5
        visual_odom.pose.covariance[7] = 0.5
        visual_odom.pose.covariance[14] = 0.00000001
        visual_odom.pose.covariance[21] = 0.0
        visual_odom.pose.covariance[28] = 0.0
        visual_odom.pose.covariance[35] = 0.5
        # visual_odom.pose.covariance = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        optical_flow_pub.publish(visual_odom)
        rate.sleep()

if __name__ == '__main__':
    main()
