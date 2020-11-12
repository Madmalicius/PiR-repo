#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from geographic_msgs.msg import GeoPoseStamped, GeoPoint
from mavros_msgs.msg import Altitude
from sensor_msgs.msg import NavSatFix
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
import csv
import os
import math
import numpy as np
from geographiclib.geodesic import Geodesic

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
        self.setp = GeoPoseStamped()
        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 2.0)
        self.local_orient = Quaternion(0.0, 0.0, 0.0, 0.0)
        self.local_coord = GeoPoint(0.0, 0.0, 0.0)
        # We will fly at a fixed altitude for now, set to 2 meters above ground
        #Initial values found from /global_position/global topic
        # initial values for setpoints
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.sp.pose.position.z = 2.0
        
        self.update = 0
        #Uncertainty for position control
        self.uncertain_dist = 0.1
        #2 degrees
        self.uncertain_rad = 0.035

        # image proccesing done
        self.proc_done = True


        ### ------------SIMULATION------------ ###
        self.simulation = False
        ### ---------------------------------- ###


        basePath = os.path.dirname(os.path.abspath(__file__))
        # open file in read mode
        with open(basePath + '/geodetic_coordinates.csv', 'r') as read_obj:
            # pass the file object to reader() to get the reader object
            csv_reader = csv.reader(read_obj)
            # Pass reader object to list() to get a list of lists
            self.coordinates = list(csv_reader)
            #print(coordinates)
        self.length=len(self.coordinates)
        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.
        
        #Set initial rotation
        self.sp.pose.orientation.x, self.sp.pose.orientation.y, self.sp.pose.orientation.z, self.sp.pose.orientation.w = self.euler_to_quaternion(0,0,math.radians(float(self.coordinates[self.update][2])))
        self.setp.pose.orientation.x, self.setp.pose.orientation.y, self.setp.pose.orientation.z, self.setp.pose.orientation.w = self.euler_to_quaternion(0,0,math.radians(float(self.coordinates[self.update][2])))
        # define the WGS84 ellipsoid
        self.geod = Geodesic.WGS84

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        self.local_orient.x = msg.pose.orientation.x
        self.local_orient.y = msg.pose.orientation.y
        self.local_orient.z = msg.pose.orientation.z
        self.local_orient.w = msg.pose.orientation.w
    ## global position callback
    def globalCb(self, msg):
        self.local_coord.latitude = msg.latitude
        self.local_coord.longitude = msg.longitude

    def altitudeCb(self, msg):
        self.local_coord.altitude = msg.amsl
    ## Drone orientation callback
    def orientCb(self,msg):
        self.local_orient.x = msg.pose.orientation.x
        self.local_orient.y = msg.pose.orientation.y
        self.local_orient.z = msg.pose.orientation.z
        self.local_orient.w = msg.pose.orientation.w
    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg
    ## Initialize flight height to ground level +2m
    def initCb(self):
        msg = rospy.wait_for_message('/mavros/altitude', Altitude)
        self.setp.pose.position.altitude = msg.amsl +2
        pos = rospy.wait_for_message('/mavros/global_position/global', NavSatFix)
        self.setp.pose.position.latitude = pos.latitude
        self.setp.pose.position.longitude = pos.longitude

    ## Conversion functions
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

    ## Starts caputuring image and returns when done
    def capture_image(self):
        rospy.wait_for_service('/camera/take_img')
        try:
            take_photo = rospy.ServiceProxy('/camera/take_img', Trigger)
            take_photo()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        return

    def proc_done_Cb(self, msg):
        self.proc_done = True

    ## Update setpoint message
    def updateSp(self):
        #Calculate distance to point
        self.distance = math.sqrt((self.local_pos.x - self.sp.pose.position.x)** 2 + (self.local_pos.y - self.sp.pose.position.y)** 2)
        y, p, r = self.quaternion_to_euler(self.local_orient.x, self.local_orient.y, self.local_orient.z, self.local_orient.w)
        self.height = abs(self.local_pos.z - self.sp.pose.position.z)
        #Calculate rotation difference
        yaw, pitch, roll = self.quaternion_to_euler(self.sp.pose.orientation.x, self.sp.pose.orientation.y, self.sp.pose.orientation.z, self.sp.pose.orientation.w)
        self.rotation = y - yaw
        self.rotation = abs((self.rotation + math.pi) % (math.pi*2) - math.pi)
        
        #print(self.height, self.local_pos.z)
        if ((self.distance <= self.uncertain_dist) and (self.rotation <= self.uncertain_rad) and (self.height <= self.uncertain_dist/2) and self.proc_done):
            
            if(self.simulation):
                self.proc_done = False
                self.capture_image()
            
            print("diller")
            self.sp.pose.position.x = float(self.coordinates[self.update][0])
            self.sp.pose.position.y = float(self.coordinates[self.update][1])
            #print(self.sp.pose.position.x, self.sp.pose.position.y, self.coordinates[self.update][2])
            self.sp.pose.orientation.x, self.sp.pose.orientation.y, self.sp.pose.orientation.z, self.sp.pose.orientation.w = self.euler_to_quaternion(0,0,math.radians(float(self.coordinates[self.update][2])))
            #print(self.sp.pose.orientation)
            self.update+=1
            #Return home and land
            if self.update >= self.length:
                self.sp.pose.position.x = 0.0
                self.sp.pose.position.y = 0.0
                self.sp.pose.position.z = 2.0
                if self.distance <= self.uncertain_dist:
                    self.sp.pose.position.x = 0.0
                    self.sp.pose.position.y = 0.0
                    self.sp.pose.position.z = 0.0
        else:
            pass
    ## Update coordinate setpoint message
    def updateSetp(self):
        #Calculate rotational difference
        yaw, pitch, roll = self.quaternion_to_euler(self.setp.pose.orientation.x, self.setp.pose.orientation.y, self.setp.pose.orientation.z, self.setp.pose.orientation.w)
        y, p, r = self.quaternion_to_euler(self.local_orient.x, self.local_orient.y, self.local_orient.z, self.local_orient.w)
        self.rotation = y - yaw
        self.rotation = abs((self.rotation + math.pi) % (math.pi*2) - math.pi)
        #Calculate distance to point
        self.g = self.geod.Inverse(self.setp.pose.position.latitude, self.setp.pose.position.longitude, self.local_coord.latitude, self.local_coord.longitude)
        self.height = abs(self.local_coord.altitude - self.setp.pose.position.altitude)
        #print(self.local_coord.altitude)
        #print("The distance is {:.3f} m.".format(self.g['s12']), self.rotation, self.height)
        
        if ((self.g['s12'] <= self.uncertain_dist) and (self.rotation <= self.uncertain_rad) and (self.height <= self.uncertain_dist/2)):
            #print(float(self.coordinates[self.update][0]))
            #print(float(self.coordinates[self.update][1]))
            print("globaldiller")
            #print(self.g['s12'])
            self.setp.pose.position.latitude = float(self.coordinates[self.update][0])
            self.setp.pose.position.longitude = float(self.coordinates[self.update][1])
            #self.setp.pose.position.altitude = self.local_coord.altitude
            self.setp.pose.orientation.x, self.setp.pose.orientation.y, self.setp.pose.orientation.z, self.setp.pose.orientation.w = self.euler_to_quaternion(0,0,math.radians(float(self.coordinates[self.update][2])))
            self.update+=1
        
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
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.orientCb)

    #Subscribe to drones gps
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, cnt.globalCb)
    rospy.Subscriber('/mavros/altitude', Altitude, cnt.altitudeCb)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    # Setpoint publisher coordinates
    setpoint_global_pub = rospy.Publisher('/mavros/setpoint_position/global', GeoPoseStamped, queue_size=1)
    # Make sure the drone is armed

    # Subscribe to image prossing done flag
    rospy.Subscriber("/camera/proc_done", Bool, cnt.proc_done_Cb)

    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        #sp_pub.publish(cnt.sp)
        setpoint_global_pub.publish(cnt.setp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()
    cnt.initCb()
    # ROS main loop
    while not rospy.is_shutdown():
        cnt.updateSetp()
        #sp_pub.publish(cnt.sp)
        setpoint_global_pub.publish(cnt.setp)
        rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
