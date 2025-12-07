#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
import numpy as np

''' Constantes '''
mtx = np.array([[813.556992, 0.000000, 328.327845],
[0.000000, 813.444712, 225.690383],
[0.000000, 0.000000, 1.000000]])

dist = np.array([0.073695, 0.053370, -0.000830, -0.000563, 0.000000])


cap = cv2.VideoCapture(-1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

rospy.init_node("no_publica_imagem")
img_pub = rospy.Publisher("/camera/rgb/image_raw", Image, queue_size= 10)

bridge = CvBridge()

if not cap.isOpened():
    exit()

while not rospy.is_shutdown():

    ret, frame = cap.read()
    if not ret:
        break
    try:
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)

        ros_image = bridge.cv2_to_imgmsg(dst, "bgr8")
        img_pub.publish(ros_image)

    except Exception as e:
        rospy.logerr(e)

    rospy.sleep()


cap.release()
