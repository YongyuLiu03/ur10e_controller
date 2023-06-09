#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point
import tf.transformations as tf
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from moveit_msgs.msg import JointConstraint, Constraints


def compute_quaternion(eef_pose: Pose):
    x = tf.unit_vector([target_point.x-eef_pose.position.x, target_point.y-eef_pose.position.y, target_point.z-eef_pose.position.z])
    z = np.array([0, 0, 1]) if not np.allclose(x, np.array([0, 0, 1])) else np.array([0, 1, 0])
    y = tf.unit_vector(np.cross(z, x))
    z = tf.unit_vector(np.cross(x, y))

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
    rospy.init_node("rotate_motion")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_names = robot.get_group_names()
    move_group = moveit_commander.MoveGroupCommander(group_names[0])
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    print("eef link: ", eef_link)

    move_group.clear_path_constraints()
    # w1_jc = JointConstraint()
    # w1_jc.joint_name = "wrist_1_joint"
    # w1_jc.position = -math.pi/2.0
    # w1_jc.tolerance_above = math.pi/2.0
    # w1_jc.tolerance_below = math.pi/2.0
    # w1_jc.weight = 1.0
    # sl_jc = JointConstraint()
    # sl_jc.joint_name = "shoulder_lift_joint"
    # sl_jc.position = 0
    # sl_jc.tolerance_above = 0
    # sl_jc.tolerance_below = math.pi/2.0
    # sl_jc.weight = 1.0

    # constraints = Constraints()
    # constraints.joint_constraints.append(w1_jc)
    # constraints.joint_constraints.append(sl_jc)
    # move_group.set_path_constraints(constraints)


    # joint_goal = move_group.get_current_joint_values()
    # print("Current joint goals: ", joint_goal[3])
    # joint_goal[0] = 0
    # joint_goal[1] = -math.pi/2.0
    # joint_goal[2] = math.pi/2.0
    # joint_goal[3] = -math.pi/2.0
    # joint_goal[4] = 0
    # joint_goal[5] = 0
    # move_group.go(joint_goal, wait=True)
    # move_group.stop()
    # input("ENTER to continue")
    # joint_goal = move_group.get_current_joint_values()
    # print("Current joint goals: ", joint_goal[3])
    # joint_goal[0] = -math.pi
    # move_group.go(joint_goal, wait=True)
    # move_group.stop()
    # input("ENTER to continue")
    # print("Current joint goals: ", joint_goal[3])
    # joint_goal[0] = math.pi
    # move_group.go(joint_goal, wait=True)
    # move_group.stop()
    # input("ENTER to continue")


    for i in range(n+1):
        eef_pose = Pose()
        if i == 0:
            eef_pose.position.x = target_point.x - 0.2
            eef_pose.position.y = target_point.y - 0.2
            eef_pose.position.z = target_point.z 
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


        if i == 0:
            print('Initializing eef pose')
        else:
            print(f"Rotate step {i}")

        execute = False
        while not execute:
            plan = move_group.plan()
            execute = "e" == input("e to excecte / other to replan ")
            

        success = move_group.go(plan[0], wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        print(f"{'executed' if success else 'failed'}")
        # print("current pose:\n", move_group.get_current_pose().pose)
        if not success:
            return
    

target_point = Point()
target_point.x = 0.6
target_point.y = 0.3
target_point.z = 0.0
n = 16
theta = 2*math.pi/n

if __name__ == '__main__':
    main()
    rospy.signal_shutdown("")
