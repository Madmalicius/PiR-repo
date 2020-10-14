#!/usr/bin/env python
import rospy


class PathPlanning():

    def __init__(self):
        pass


if __name__ == '__main__':
    pp = PathPlanning()

    rospy.init_node("path_planning")

    # Loop
    while(not rospy.is_shutdown()):
        pass
