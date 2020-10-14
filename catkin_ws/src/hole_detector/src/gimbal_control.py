#!/usr/bin/env python
import rospy


class GimbalControl():

    def __init__(self):
        pass


if __name__ == '__main__':
    gc = GimbalControl()

    rospy.init_node("gimbal_control")

    # Loop
    while(not rospy.is_shutdown()):
        pass
