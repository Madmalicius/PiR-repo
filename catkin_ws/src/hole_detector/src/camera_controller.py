#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CameraController():

    def __init__(self):
        pass


if __name__ == '__main__':
    cc = CameraController()

    rospy.init_node("camera_controller")

    # Loop
    while(not rospy.is_shutdown()):
        pass
