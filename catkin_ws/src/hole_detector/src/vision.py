#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, NavSatFix


class HoleDetector():

    def __init__(self):
        pass

    def detect(self, img):
        return False


current_pos = None
positions = []
img_msgs = []

# On new image, save image message and current gps position
def image_callback(msg):
    global img_msgs, positions
    img_msgs.append(msg)
    positions.append(current_pos)

def gps_callback(msg):
    current_pos = (msg.latitude, msg.longitude, msg.altitude)

if __name__ == '__main__':
    hd = HoleDetector()
    bridge = CvBridge()

    rospy.init_node("holedetector_vision")
    rospy.Subscriber("/camera/image_raw", Image, image_callback)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)

    rate = rospy.Rate(5)

    # Loop
    while(not rospy.is_shutdown()):
        # Wait for received image
        while current_img_msg is None:
            rate.sleep()
        # Convert message to image
        img = bridge.imgmsg_to_cv2(current_img_msg, "bgr8")
        # Reset message variable
        current_img_msg = None
        # Run detection
        fault = hd.detect(img)
        # If fault, publish image and 
        if fault:
            print("Fault detected!")
            # TODO Publish image with position
