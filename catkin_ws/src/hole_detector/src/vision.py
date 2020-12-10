#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped
from camera_controller import CameraController
import math
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from GPSPhoto import gpsphoto
from PIL import Image
import imutils
import time
<<<<<<< HEAD
import os
=======
from warp_image_func import warp_image 
>>>>>>> 004bbeff9770d6903d4f3b4ffd3973b48dc1c41d


class HoleDetector():

    def __init__(self):
        pass

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
current_rot = None
proc_data = []

cam = None

imgDirPath = "/home/ubuntu/fence_imgs"
faultCount = 0
cap_pub = None

# On new image, save image message and current gps position
def image_callback(msg):
    global proc_data, cam, cap_pub
    print("taking image")
    img = cam.capture()
    proc_data.append((img, current_pos, current_rot))
    print("Image taken")
    cap_done = Bool()
    cap_done.data = True
    cap_pub.publish(cap_done)

def gps_callback(msg):
    global current_pos
    current_pos = (msg.latitude, msg.longitude, msg.altitude)

def quaternions_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return [yaw, pitch, roll]

def pose_callback(msg):
    global current_rot
    quat = msg.pose.orientation
    current_rot = quaternions_to_euler(quat.x, quat.y, quat.z, quat.w)

def ImgGPSCombiner(pos,imgPath):
    imgPathTagged = imgPath + "Tag.jpg"

    photo = gpsphoto.GPSPhoto(imgPath+".jpg")

    info = gpsphoto.GPSInfo((pos[0], pos[1]))

    # Modify GPS Data
    photo.modGPSData(info, imgPathTagged)
    os.remove(imgPath+".jpg")

if __name__ == '__main__':
    print("Vision Node")
    cam = CameraController()
    hd = HoleDetector()

    rospy.init_node("holedetector_vision")
    rospy.Subscriber("/camera/take_img", Bool, image_callback)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
    cap_pub = rospy.Publisher("/camera/cap_done", Bool, queue_size=10)
    done_pub = rospy.Publisher("/camera/proc_done", Bool, queue_size=10)

    K = np.array([4076.983308901881,0.0,1959.7405511739269,0.0,4068.5673929816157,1630.9662530185433,0.0,0.0,1.0])
    K = K.reshape((3,3))
    dist = np.array([-0.196564133150198,0.40723829946509277,-4.523529625230164,15.295234865994818])

    img_width = 4064
    img_height = 3040
    mx, my = cv2.fisheye.initUndistortRectifyMap(K, dist, None, K, (img_width, img_height), cv2.CV_32FC1)

    faultCount = 0

    time.sleep(2)

    rate = rospy.Rate(15)
    cnt = 0

    # Loop
    while(not rospy.is_shutdown()):
        print("Waiting for image")
        # Wait for received image
        while len(proc_data) == 0 and (not rospy.is_shutdown()):
            rate.sleep()
        print("Processing image")
        img, pos, rot = proc_data.pop(0)

        # Undistort
        img = cv2.remap(img, mx, my, cv2.INTER_LINEAR)

        # Correct pitch
        img = warp_image(img, np.degrees(rot[1]) - 20)

        # Correct roll
        img = imutils.rotate_bound(img, np.degrees(rot[2]))

        cnt += 1
        
        # Run detection
        fault = True#hd.detect(img)
        # If fault, publish image and 
        if fault:
            print("Fault detected!")
            imgPath = imgDirPath + "/Err"+str(faultCount)
            cv2.imwrite(imgPath+".jpg",img)
            ImgGPSCombiner(pos,imgPath)
            faultCount += 1

        proc_done = Bool()
        proc_done.data = True
        done_pub.publish(proc_done)
