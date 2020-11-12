#!/usr/bin/env python
import cv2
import io
#import rospy
import numpy as np
from sensor_msgs.msg import Image
import os
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

class CameraController():

    def __init__(self):
        #self.cap = cv2.VideoCapture(0)
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3280) #4056
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2464) #3040
        self.cam = PiCamera()
        self.cam.resolution = (4056, 3040)
        self.cam.framerate = 5
        self.cam.sharpness = 80
        self.cam.iso = 0 #800
        self.cam.shutter_speed = 0      # shutter, or 0 for auto
        self.cam.image_effect = 'none'  # sketch, denoise, cartoon
        self.cam.exposure_mode = 'auto'  # auto, night, spotlight, sports, snow, beach, antishake, fireworks
        self.cam.exposure_compensation = 0  #-25 - 25
        self.cam.contrast = 0           # -100 - 100
        self.cam.awb_mode = 'auto'      # off, auto, sunlight, cloudy, shade, tungsten, flourescent, incandescent, flash, horizon
        self.cam.image_denoise = True
        self.cam.meter_mode = 'backlit'
        self.cam.saturation = 0         # -100 - 100
        time.sleep(2)

    def capture(self):
        img = np.empty((4064 * 3040 * 3), dtype=np.uint8)
        self.cam.capture(img, format='bgr')
        img = img.reshape((3040, 4064, 3))

        #data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        #img = cv2.imdecode(data, 1)
        #ret, img = self.cap.read()
        #if not ret:
        #    print("Capture failed")
        #img = self.cam.capture()
        return img

if __name__ == '__main__':
    cc = CameraController()
    #rospy.init_node("camera_controller")

    t = time.time()
    img = cc.capture()
    print("Get img took ", time.time() - t, " seconds")
    t = time.time()
    cv2.imwrite("/home/ubuntu/fence_imgs/img_1.jpg", img)
    print("Writing img took ", time.time() - t, " seconds")

    # Loop
    #while(not rospy.is_shutdown()):
    #    pass
