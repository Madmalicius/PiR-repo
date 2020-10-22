#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Quaternion
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from uncertainties import ufloat
import csv
import os
import math
import numpy as np

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Autoland Mode could not be set."%e

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PoseStamped()
        # set the flag to use position setpoints and yaw angle
        #self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        #self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 2.0
        # update the setpoint message with the required altitude
        self.sp.pose.position.z = self.ALT_SP
        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 2.0)
        self.local_orient = Quaternion(0.0, 0.0, 0.0, 0.0)

        # initial values for setpoints
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0

        self.sp.pose.orientation.x, self.sp.pose.orientation.y, self.sp.pose.orientation.z, self.sp.pose.orientation.w = self.euler_to_quaternion(0,0,0)
        self.update = 0
        #Uncertainty for position control
        self.uncertain_dist = 0.1
        #2 degrees
        self.uncertain_rad = 0.035

        basePath = os.path.dirname(os.path.abspath(__file__))
        # open file in read mode
        with open(basePath + '/output_coordiantas.csv', 'r') as read_obj:
            # pass the file object to reader() to get the reader object
            csv_reader = csv.reader(read_obj)
            # Pass reader object to list() to get a list of lists
            self.coordinates = list(csv_reader)
            #print(coordinates)
        self.length=len(self.coordinates)
        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.
	# Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        self.local_orient.x = msg.pose.orientation.x
        self.local_orient.y = msg.pose.orientation.y
        self.local_orient.z = msg.pose.orientation.z
        self.local_orient.w = msg.pose.orientation.w
    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def euler_to_quaternion(self, roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

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

    ## Update setpoint message
    def updateSp(self):
        #Calculate distance to point
        self.distance = math.sqrt((self.local_pos.x - self.sp.pose.position.x)** 2 + (self.local_pos.y - self.sp.pose.position.y)** 2)
        self.rotation = self.quaternion_to_euler(self.local_orient.x, self.local_orient.y, self.local_orient.z, self.local_orient.w)
        print(self.rotation)
        if ((self.distance <= self.uncertain_dist) and (abs(self.rotation[0] - math.radians(float(self.coordinates[self.update][2]))) <= self.uncertain_rad)):
            print("diller")
            self.sp.pose.position.x = float(self.coordinates[self.update][0])
            self.sp.pose.position.y = float(self.coordinates[self.update][1])
            self.sp.pose.orientation.x, self.sp.pose.orientation.y, self.sp.pose.orientation.z, self.sp.pose.orientation.w = self.euler_to_quaternion(0,0,math.radians(float(self.coordinates[self.update][2])))
            print(self.sp.pose.orientation)
            self.update+=1
            #Return home
            if self.update >= self.length:
                self.sp.pose.position.x = 0.0
                self.sp.pose.position.y = 0.0
                self.sp.pose.position.z = 0.0
        else:
            pass


# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)


    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    # set in takeoff mode and takeoff to default altitude (3 m)
    # modes.setTakeoff()
    # rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()

    # ROS main loop
    while not rospy.is_shutdown():
        
        cnt.updateSp()
        sp_pub.publish(cnt.sp)
        rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
