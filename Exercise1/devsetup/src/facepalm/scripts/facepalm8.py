#!/usr/bin/env python

import rospy, time
import subprocess #import Popen

from std_msgs.msg import String

gazebo = subprocess.Popen("roslaunch reemc_gazebo reemc_empty_world.launch".split())

gazebo = subprocess.Popen("roslaunch reemc_controller_configuration joint_trajectory_controllers.launch".split())

rospy.init_node('facepalmer', anonymous=True)
rate = rospy.Rate(10)

while not rospy.is_shutdown()
    rate.sleep()
