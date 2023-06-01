#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point
import tf.transformations as tf
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

def compute_quaternion(eef_pose: Pose, target_point: Point):
    x = tf.unit_vector([target_point.x-eef_pose.position.x, target_point.y-eef_pose.position.y, target_point.z-eef_pose.position.z])
    z = np.array([0, 0, 1]) if not np.allclose(x, np.array([0, 0, 1])) else np.array([0, 1, 0])
    y = tf.unit_vector(np.cross(z, x))
    z = tf.unit_vector(np.cross(x, y))

    rotation_matrix = np.array([x, y, z]).transpose()
    r = R.from_matrix(rotation_matrix)
    quaternion = r.as_quat()
    return quaternion


center_point = Point()
center_point.x = 0.5
center_point.y = 0.5
center_point.z = 0.0

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("square_motion")

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_names = robot.get_group_names()
move_group = moveit_commander.MoveGroupCommander(group_names[0])
planning_frame = move_group.get_planning_frame()
flange_link = move_group.get_end_effector_link()

h = 4

target_pose = Pose()
target_pose.position.x = 0.5
target_pose.position.y = 0.5
target_pose.position.z = 0.0

x = np.array([1, 0, 0])
z = np.array([0, 1, 0])
y = np.array([0, 0, 1])
rotation_matrix = np.array([x, y, z]).transpose()
r = R.from_matrix(rotation_matrix)
quaternion = r.as_quat()
# target_pose.orientation.x = quaternion[0]
# target_pose.orientation.y = quaternion[1]
# target_pose.orientation.z = quaternion[2]
# target_pose.orientation.w = quaternion[3]

move_group.set_pose_target(target_pose)
print("Initializing printer")
success = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()
print(f"{'executed' if success else 'failed'}")


for j in range(h):
    target_pose.position.z += 0.1
    for i in range(1, 5):
        if i == 1:
            target_pose.position.x += 0.2
        elif i == 2:
            target_pose.position.y += 0.2
        elif i == 3:
            target_pose.position.x -= 0.2
        elif i == 4:
            target_pose.position.y -= 0.2
        waypoints = []
        waypoints.append(move_group.get_current_pose().pose)
        waypoints.append(target_pose)
        fraction = 1.0
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        print(target_pose)