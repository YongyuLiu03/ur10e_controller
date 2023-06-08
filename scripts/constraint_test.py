#!/usr/bin/env python3 
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import DisplayTrajectory, JointConstraint, Constraints
import tf.transformations as tf
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("constraint_test")

    robot = moveit_commander.RobotCommander()
    group_names = robot.get_group_names()
    move_group = moveit_commander.MoveGroupCommander(group_names[0])

    move_group.clear_path_constraints()

    i = 0
    joint_names = move_group.get_active_joints()
    print("joint names: ", joint_names)
    joint_goal = move_group.get_current_joint_values()
    print("Current joint goals: ", joint_goal)
    joint_goal[i] = -math.pi

    # return
    # joint_goal[i] = -math.pi/2.0
    # joint_goal[2] = math.pi/2.0
    # joint_goal[5] = 0
    move_group.go(joint_goal, wait=True)
    # move_group.stop()
    joint_goal = move_group.get_current_joint_values()
    print("Current joint goals: ", joint_goal[i])
    input("ENTER to continue")
    joint_goal[i] = -math.pi/2
    move_group.go(joint_goal, wait=True)
    # move_group.stop()
    print("Current joint goals: ", joint_goal[i])
    input("ENTER to continue")
    joint_goal = move_group.get_current_joint_values()
    joint_goal[i] = 0
    move_group.go(joint_goal, wait=True)
    # move_group.stop()
    print("Current joint goals: ", joint_goal[i])
    input("ENTER to continue")
    joint_goal = move_group.get_current_joint_values()
    joint_goal[i] = math.pi/2
    move_group.go(joint_goal, wait=True)
    # move_group.stop()
    print("Current joint goals: ", joint_goal[i])
    input("ENTER to continue")
    joint_goal = move_group.get_current_joint_values()
    joint_goal[i] = math.pi
    move_group.go(joint_goal, wait=True)
    # move_group.stop()
    print("Current joint goals: ", joint_goal[i])
    input("ENTER to continue")


    w3_jc = JointConstraint()
    w3_jc.joint_name = "wrist_3_joint"
    w3_jc.position = math.pi/2.0
    w3_jc.tolerance_above = math.pi/2.0
    w3_jc.tolerance_below = math.pi/2.0
    w3_jc.weight = 1.0
    constraints = Constraints()
    constraints.joint_constraints.append(w3_jc)
    # move_group.set_path_constraints(constraints)

if __name__ == "__main__":
    main()