#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from moveit_msgs.msg import DisplayTrajectory, JointConstraint, Constraints
import tf.transformations as tf
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import copy
import tf2_ros


def plan_and_execute(target_pose):
    move_group.set_pose_target(target_pose)
    success = move_group.go(wait=True)
    move_group.clear_pose_targets()
    if not success:
        exit()
    return 

def get_transform(): 
    transform = None
    rate = rospy.Rate(10.0)
    while not transform:
        try:
            transform = tfBuffer.lookup_transform('base_link', 'camera_color_optical_frame', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

    return transform

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("camera_motion")
    pub = rospy.Publisher("transformation", TransformStamped, queue_size=10)

    global move_group
    global tfBuffer
    global listener

    robot = moveit_commander.RobotCommander()
    group_names = robot.get_group_names()
    move_group = moveit_commander.MoveGroupCommander(group_names[0])
    move_group.clear_path_constraints()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    

    move_group.set_planning_time(20)
    move_group.set_num_planning_attempts(5)
    
    init_pose = Pose()
    init_pose.position.x = float(0.5)
    init_pose.position.y = float(0)
    init_pose.position.z = float(0.05)
    length = 0.1
    rate = rospy.Rate(120)

    rot_z_90 = R.from_rotvec(np.radians(90)*np.array([0, 0, 1]))
    init_r = R.from_rotvec(np.radians(0)*np.array([0, 0, 1]))
    cur_r = init_r
    cur_pose = init_pose
    cur_pose.position.y += 0.5*length

    plan_and_execute(cur_pose)
    pub.publish(get_transform())
    rate.sleep()

    for i in range(4):

        cur_r = rot_z_90 * cur_r
        cur_quat = cur_r.as_quat()
        
        if i == 0:
            cur_pose.position.y -= 0.5*length
            cur_pose.position.x += 0.5*length
        elif i == 1:
            cur_pose.position.y += 0.5*length
            cur_pose.position.x += 0.5*length
        elif i == 2:
            cur_pose.position.x -= 0.5*length
            cur_pose.position.y += 0.5*length
        elif i == 3:
            cur_pose.position.x -= 0.5*length
            cur_pose.position.y -= 0.5*length

        cur_pose.orientation.x = cur_quat[0]
        cur_pose.orientation.y = cur_quat[1]
        cur_pose.orientation.z = cur_quat[2]
        cur_pose.orientation.w = cur_quat[3]
        
        plan_and_execute(cur_pose)
        pub.publish(get_transform())
        rate.sleep()


if __name__ == "__main__":
    main()