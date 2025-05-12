#!/usr/bin/env python
# -*- encoding: utf-8 -*-

#from __future__ import print_function
import rospy, time
import subprocess #import Popen

from numpy import pi

from std_msgs.msg import String

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
)

from trajectory_msgs.msg import (     
    JointTrajectory,
    JointTrajectoryPoint,
)



#gazebo = subprocess.Popen("roslaunch reemc_gazebo reemc_empty_world.launch".split())

#reemc_controller = subprocess.Popen("roslaunch reemc_controller_configuration joint_trajectory_controllers.launch".split())

rospy.init_node('facepalmer', anonymous=True)
rate = rospy.Rate(1)

controller='/right_arm_controller'

import actionlib

client=actionlib.SimpleActionClient(controller + "/follow_joint_trajectory", FollowJointTrajectoryAction)
client.wait_for_server()


right_arm_joint_names = rospy.get_param(controller+"/joints")
print(right_arm_joint_names)

time.sleep(3)
print("let's start")

target = JointTrajectoryPoint()
#target.positions = [0.1]*len(right_arm_joint_names)
target.positions=[
    pi/2, # rotation forward shoulder
    0.2,    # rotation sidewards shoulder
    -1.0,    # rotation arm-internal
    pi/2+0.4,  # rotation ellbow
    2,
    -0.1,
    0
]
target.time_from_start = rospy.Duration(5)  # seconds

goal = FollowJointTrajectoryGoal()
goal.trajectory.points.append(target)
goal.goal_time_tolerance=rospy.Duration(0.5)
goal.trajectory.joint_names=right_arm_joint_names

i=0
while not rospy.is_shutdown() and i < 3:
    client.send_goal(goal)
    client.wait_for_result(timeout=rospy.Duration(60))
    #rate.sleep()
    print("Goal sent")
    time.sleep(10)
    i+=1




"""
while not rospy.is_shutdown():
    print("Gucci")
    rate.sleep()
    
    right_arm_joint_names = rospy.get_param("/right_arm_controller/joints")
    print(right_arm_joint_names)
"""

"""
#import argparse
#import sys

from copy import copy

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)

import moveit_commander

from moveit_msgs.srv import GetPlannerParams
import moveit_msgs.srv

from reemc_joint_trajectory_player.joint_trajectory_client import JointTrajectoryClient
from reemc_joint_trajectory_player.trajectory_editor import TrajectoryEditor

def set_longest_valid_segment_fraction(string_value):
    rospy.wait_for_service('set_planner_params')
    set_planner_params = rospy.ServiceProxy('set_planner_params', moveit_msgs.srv.SetPlannerParams)
    req = moveit_msgs.srv.SetPlannerParamsRequest()
    req.planner_config = "RRTConnectkConfigDefault"
    req.group = "left_arm"
    req.params = moveit_msgs.msg.PlannerParams(keys=["longest_valid_segment_fraction"], values=[string_value])
    set_planner_params(req)

def main():
    print("Initializing node... ")
    rospy.init_node("joint_trajectory_client_interpolation_test")

    reemc_left_arm_moveit = moveit_commander.MoveGroupCommander("left_arm")

    planner_id = "RRTConnectkConfigDefault"
    reemc_left_arm_moveit.set_planner_id(planner_id)
    set_longest_valid_segment_fraction("0.01")

    left_arm_config_1 = [0.2, 0.7, 0.1, 0.1, 0.1, 0.1, 0.1]
    left_arm_joint_names = rospy.get_param("/right_arm_controller/joints")

    te = TrajectoryEditor()

    jtc = JointTrajectoryClient('left_arm_controller', left_arm_joint_names)
    rospy.on_shutdown(jtc.stop)
    
    print("Moving to 'left_arm_config_1' position")
    plan = reemc_left_arm_moveit.plan(left_arm_config_1)

    traj = te.retime_traj_and_remove_v_a_e(plan.joint_trajectory, 3.0)
    print("Length of trajectory: " + str(len(traj.points)))

    #extract the joint_trajectory from the plan and send it to the JointTrajectoryClient
    jtc.add_full_trajectory(traj)

    jtc.start()
    jtc.wait(10.0)

    #print(jtc.result())
    
    #jtc.stop()
    print("Try to stop trajectory")
    rospy.sleep(2.0)

    set_longest_valid_segment_fraction("0.0005")

    # try:
    #     set_planner_params("", 'left_arm', req.params, True)
    # except rospy.ServiceException as e:
    #     rospy.logerr('Failed to get params: %s', e)

    left_arm_config_2 = [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    print("Moving to 'left_arm_config_2' position")
    plan = reemc_left_arm_moveit.plan(left_arm_config_2)

    traj = te.retime_traj_and_remove_v_a_e(plan.joint_trajectory, 3.0)
    print("Length of trajectory: " + str(len(traj.points)))

    #extract the joint_trajectory from the plan and send it to the JointTrajectoryClient
    jtc.add_full_trajectory(traj)

    jtc.start()
    jtc.wait(10.0)

    #print(jtc.result())
    
    jtc.stop()
    print("Try to stop trajectory")

    print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":

    #simple test of the JointTrajectoryClient
    main()
"""
