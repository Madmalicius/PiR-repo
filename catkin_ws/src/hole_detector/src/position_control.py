#!/usr/bin/env python
import rospy


class PositionControl():

    def __init__(self):
        pass


if __name__ == '__main__':
    pc = PositionControl()

    rospy.init_node("position_control")

    # Loop
    while(not rospy.is_shutdown()):
        pass
