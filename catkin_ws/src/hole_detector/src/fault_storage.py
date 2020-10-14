#!/usr/bin/env python
import rospy


class FaultStorage():

    def __init__(self):
        pass


if __name__ == '__main__':
    fs = FaultStorage()

    rospy.init_node("fault_storage")

    # Loop
    while(not rospy.is_shutdown()):
        pass
