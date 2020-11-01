#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, NavSatFix
from camera_controller import CameraController
from std_srvs.srv import Trigger


class HoleDetector():

    def __init__(self):
        

    def detect(self, img):


current_pos = None
proc_data = []

cam = None

# On new image, save image message and current gps position
def image_callback(msg):
    global proc_data
    if cam is None:
        return
    img = cam.capture()
    proc_data.append((img, current_pos))

def gps_callback(msg):
    current_pos = (msg.latitude, msg.longitude, msg.altitude)

if __name__ == '__main__':
    cam = CameraController()
    hd = HoleDetector()

    rospy.init_node("holedetector_vision")
    rospy.Service("/camera/take_img", Trigger, image_callback)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)

    time.sleep(2)

    rate = rospy.Rate(15)

    # Loop
    while(not rospy.is_shutdown()):
        print("Waiting for image")
        # Wait for received image
        while len(proc_data) == 0:
            rate.sleep()
        print("Processing image")
        img, pos = proc_data.pop(0)
        proc_data
        # Run detection
        fault = hd.detect(img)
        # If fault, publish image and 
        if fault:
            print("Fault detected!")
            # TODO Save image with position
