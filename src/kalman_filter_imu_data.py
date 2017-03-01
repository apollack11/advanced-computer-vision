#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Imu

class data_filter:
    def __init__(self):
        # subcribe to IMU data from Jackal
        self.imu_data_sub = rospy.Subscriber("/imu/data", Imu, self.callback)
        # will also need a subscriber to camera data
        # camera_sub = rospy.Subscriber("TOPIC NAME", TOPIC, callback)
        # publish filtered_data
        self.filtered_data_pub = rospy.Publisher('/filtered_data', Imu, queue_size=10)

        self.filtered_data = Imu()

        self.meas=[]
        self.pred=[]
        self.mp = np.zeros((6,1), np.float32) # measurement
        self.tp = np.zeros((6,1), np.float32) # tracked / prediction
        print self.mp.shape
        print self.tp.shape

        # Kalman filter setup
        # initialize Kalman filter with 6 dynamic states (IMU data) and 6 measured states (camera data)
        self.kalman = cv2.KalmanFilter(6,6)
        self.kalman.measurementMatrix = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]],np.float32)
        self.kalman.transitionMatrix = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]],np.float32)
        self.kalman.processNoiseCov = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]],np.float32) * 0.03

        print self.kalman.measurementMatrix.shape
        print self.kalman.transitionMatrix.shape
        print self.kalman.processNoiseCov.shape

    def callback(self,data):
        # while True:
        #     kalman.correct(mp) # replace mp with measured data
        #     tp = kalman.predict() # tp will be the new IMU data
        #     pred.append((int(tp[0]),int(tp[1])))
        #     paint()
        #     cv2.imshow("kalman",frame)
        #     k = cv2.waitKey(30) &0xFF
        #     if k == 27: break
        #     if k == 32: reset()

        self.mp =
        self.kalman.correct(self.mp)
        self.tp = self.kalman.predict()
        print self.tp


        self.filtered_data = data
        # TODO: Name the variable to hold data
        self.filtered_data_pub.publish(self.filtered_data)

def main():
    rospy.init_node('kalman_filter_imu_data', anonymous=True)

    df = data_filter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
    print "OpenCV Major Version:",major_ver
    main()
