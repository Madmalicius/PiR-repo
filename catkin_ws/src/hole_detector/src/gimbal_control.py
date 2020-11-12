#!/usr/bin/env python


import rospy
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
import numpy as np
import math

class GimbalControl():

    def __init__(self):
        

        self.degMax = 180.0
        self.degMin = 0.0

        #self.yaw_measure = None     # z-axis
        self.pitch_measure = None   # y-axis 
        self.roll_measure = None    # x-axis

        self.pitch_sp = 0.0        # y-axis - set point ----  THIS ISN'T 0.0 -- this should be something else
        self.roll_sp = 0.0         # x-axis - set point ----  THIS ISN'T 0.0 -- this should be something else


        self.pitch_err = None        # y-axis 
        self.roll_err = None         # x-axis


    



        rospy.init_node("gimbal_node",anonymous=True)

        # Subscribe to drone's local position
        rospy.Subscriber('mavros/local_position/pose', PoseStamped,self.getOriantation)

        rospy.spin()


    def getOriantation(self,msg):
        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w

        __ , self.pitch, self.roll = self.quaternion_to_euler(x, y, z, w) # yaw shouldn't be used for the gimbal

        self.pitch_err = self.pitch_sp - self.pitch
        self.roll_err = self.roll_sp - self.roll

        pichPWM = self.normalizeData_to_PWMoutput(self.pitch_err)
        rollPWM = self.normalizeData_to_PWMoutput(self.roll_err)

    def quaternion_to_euler(self, x, y, z, w):

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


    def normalizeData_to_PWMoutput(self,data):
        
        z = (data-self.degMin)/(self.degMax-self.degMin)*100 # output range from 0 - 100 [PWM]

        return z




if __name__ == '__main__':
    gc = GimbalControl()

    #rospy.init_node("gimbal_control")

    # Loop
    #while(not rospy.is_shutdown()):
    #    pass
