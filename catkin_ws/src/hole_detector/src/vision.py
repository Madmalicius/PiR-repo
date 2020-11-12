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
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        kern = np.array([[-2, -1, 0], [-1, 0, 1], [0, 1, 2]])
        filh1 = cv2.filter2D(h, -1, kern)
        fils1 = cv2.filter2D(s, -1, kern)
        filv1 = cv2.filter2D(v, -1, kern)
        '''kern = np.array([[2, 1, 0], [1, 0, -1], [0, -1, -2]])
        filh2 = cv2.filter2D(h, -1, kern)
        fils2 = cv2.filter2D(s, -1, kern)
        filv2 = cv2.filter2D(v, -1, kern)
        kern = np.array([[0, 1, 2], [-1, 0, 1], [-2, -1, 0]])
        filh3 = cv2.filter2D(h, -1, kern)
        fils3 = cv2.filter2D(s, -1, kern)
        filv3 = cv2.filter2D(v, -1, kern)'''
        kern = np.array([[0, -1, -2], [1, 0, -1], [2, 1, 0]])
        filh4 = cv2.filter2D(h, -1, kern)
        fils4 = cv2.filter2D(s, -1, kern)
        filv4 = cv2.filter2D(v, -1, kern)

        fact = 1 / 2
        filh = filh1 * fact + filh4 * fact# + filh2 * fact + filh3 * fact
        fils = fils1 * fact + fils4 * fact# + fils2 * fact + fils3 * fact
        filv = filv1 * fact + filv2 * fact# + filv3 * fact + filv4 * fact
        fil = 0.5 * filv + 0.5 * filh + 0.4 * fils
        fil = cv2.convertScaleAbs(fil)

        cv2.imshow("FiltH", cv2.convertScaleAbs(filh))
        cv2.imshow("FiltS", cv2.convertScaleAbs(fils))
        cv2.imshow("FiltV", cv2.convertScaleAbs(filv))
        cv2.imshow("FiltCombined", fil)

        
        kernel = self.create_kernel(self.kernel_size, 10)
        filt = cv2.filter2D(fil, -1, kernel) #

        cv2.imshow("Filt", filt)

        ret, filt = cv2.threshold(filt, 80, 255, cv2.THRESH_BINARY)
        struc = cv2.getStructuringElement(cv2.MORPH_RECT, (80, 80))
        struc = struc * 0.9
        #res = cv2.erode(filt, struc, (-1, -1))
        #res = cv2.dilate(res, struc, (-1, -1))
        res = cv2.filter2D(filt, -1, struc)
        res = cv2.bitwise_not(res)

        struc = cv2.getStructuringElement(cv2.MORPH_RECT, (17, 17))

        res = cv2.erode(res, struc, (-1, -1))

        _,contours,_ = cv2.findContours(res, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        fault_found = False

        if len(contours) != 0:
            #find the biggest area of the contour
            c = max(contours, key = cv2.contourArea)

            x,y,w,h = cv2.boundingRect(c)

            #cv2.rectangle(self.img,(x,y),(x+w,y+h),(0,0,255),2)

            if c > self.min_fault_area:
                print("Fault found!")
                fault_found = True

        cv2.imshow("Res", res)

        return fault_found

    def create_kernel(self, size, thickness):
        plus_val = 0.01
        minus_val = 0.015
        img_size = int(size * 1.2)
        margin = int((img_size - size) / 2)
        img = np.zeros((img_size, img_size), np.float32)
        img -= minus_val
        
        #cv2.line(img, (0, margin), (img_size - 1, margin), (plus_val), thickness)
        #cv2.line(img, (0, size + margin), (img_size - 1, size + margin), (plus_val), thickness)
        #cv2.line(img, (margin, 0), (margin, img_size - 1), (plus_val), thickness)
        #cv2.line(img, (size + margin, 0), (size + margin, img_size), (plus_val), thickness)
        cv2.line(img, (0, 0), (img_size - 1, img_size - 1), (plus_val), thickness)
        cv2.line(img, (img_size - 1, 0), (0, img_size - 1), (plus_val), thickness)

        cv2.imshow("Kernel", img)
        #cv2.waitKey(10)

        #k_sum = cv2.sumElems(img)[0]
        #img = img / (k_sum)

        return img


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
