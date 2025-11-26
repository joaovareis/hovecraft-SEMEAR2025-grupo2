#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

img_pub = rospy.Publisher("/camera/rgb/image_raw", Image, queue_size= 10)

if not cap.isOpened():
    exit()

while True:

    ret, frame = cap.read()
    if not ret:
        break


cap.release()
