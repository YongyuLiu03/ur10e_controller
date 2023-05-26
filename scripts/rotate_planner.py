#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point
import tf.transformations as tf
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R


def compute_quaternion(eef_pose: Pose):
    z = tf.unit_vector([target_point.x-eef_pose.position.x, target_point.y-eef_pose.position.y, target_point.z-eef_pose.position.z])
    y = np.array([0, 1, 0]) if abs(np.dot(z, [0, 1, 0])) != 1 else np.array([0, 0, 1])
    x = tf.unit_vector(np.cross(y, z))
    y = tf.unit_vector(np.cross(z, x))

    rotation_matrix = np.array([x, y, z]).transpose()
    r = R.from_matrix(rotation_matrix)
    quaternion = r.as_quat()
    return quaternion

def compute_point(x1, y1):
    x1_ = x1 - target_point.x
    y1_ = y1 - target_point.y
    x2_ = x1_ * math.cos(theta) - y1_ * math.sin(theta)
    y2_ = x1_ * math.sin(theta) + y1_ * math.cos(theta)
    x2 = x2_ + target_point.x
    y2 = y2_ + target_point.y
    return x2, y2

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("rotate_planner")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_names = robot.get_group_names()
    move_group = moveit_commander.MoveGroupCommander(group_names[0])
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    print("eef link: ", eef_link)

    for i in range(n+1):
        eef_pose = Pose()
        if i == 0:
            eef_pose.position.x = target_point.x - 0.2
            eef_pose.position.y = target_point.y - 0.2
            eef_pose.position.z = target_point.z + 0.2
        else: 
            eef_pose = move_group.get_current_pose().pose
            x, y = compute_point(eef_pose.position.x, eef_pose.position.y)
            eef_pose.position.x, eef_pose.position.y = x, y

        quaternion = compute_quaternion(eef_pose)
        eef_pose.orientation.x = quaternion[0]
        eef_pose.orientation.y = quaternion[1]
        eef_pose.orientation.z = quaternion[2]
        eef_pose.orientation.w = quaternion[3]

        move_group.set_pose_target(eef_pose)
        joint_goals = move_group.get_joint_value_target()

        if i == 0:
            print('Initializing eef pose')
        else:
            print(f"Rotate step {i}")

        if chomp:
            success = move_group.go(joint_goals, wait=True)
        else: 
            success = move_group.go(wait=True)
        move_group.clear_pose_targets()

        print(f"{'executed' if success else 'failed'}")
        print("current pose:\n", move_group.get_current_pose().pose)
        if not success:
            rospy.signal_shutdown("")
            exit()
    
    rospy.signal_shutdown("")

target_point = Point()
target_point.x = 0.6
target_point.y = 0.6
target_point.z = 0.1
n = 16
chomp = False
theta = 2*math.pi/n

if __name__ == '__main__':
    main()