#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import numpy as np

# from center_points.center_points.lib import PointCommunicator
# from center_points.lib import PointCommunicator
from center_points.lib import PointCommunicator

def calc_com():
    com = np.random.randint(-3, 3, 3)
    return com

if __name__ == '__main__':
    communicator = PointCommunicator('center_of_mass')
    while not rospy.is_shutdown():
        com = calc_com()
        try:
            communicator.publish_point(com)
        except rospy.ROSInterruptException:
            pass


rospy.init_node('com_calc', anonymous=True)
rate = rospy.Rate(1)

import actionlib
