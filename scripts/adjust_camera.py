#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import tf.transformations as tf_tr
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros

def plan_and_execute(target_pose):
    move_group.set_pose_target(target_pose)
    success = move_group.go(wait=True)
    move_group.clear_pose_targets()
    if not success:
        exit()
    return 




def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("camera_motion")

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
    
    T_cam_eef = None
    rate = rospy.Rate(10.0)
    while not T_cam_eef:
        try:
            T_cam_eef = tfBuffer.lookup_transform('printer_link', 'camera_color_optical_frame', rospy.Time()).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
    q = T_cam_eef.rotation
    r = R.from_quat([q.x, q.y, q.z, q.w])
    matrix = r.as_matrix()
    t = T_cam_eef.translation
    translation = np.array([t.x, t.y, t.z])
    T_cam_eef = np.eye(4)
    T_cam_eef[:3, :3] = matrix
    T_cam_eef[:3, 3] = translation

    cam_pose = Pose()
    cam_pose.position.x = 0.5
    cam_pose.position.y = 0.2
    cam_pose.position.z = 0.4
    cam_r = R.from_rotvec(np.radians(180)*np.array([0, 1, 0]))
    cam_quat = cam_r.as_quat()
    cam_pose.orientation.x = cam_quat[0]
    cam_pose.orientation.y = cam_quat[1]
    cam_pose.orientation.z = cam_quat[2]
    cam_pose.orientation.w = cam_quat[3]
    cam_mat = cam_r.as_matrix()
    print(cam_mat)
    cam_trans = np.array([cam_pose.position.x, cam_pose.position.y, cam_pose.position.z])
    T_cam = np.eye(4)
    T_cam[:3, :3] = cam_mat
    T_cam[:3, 3] = cam_trans

    T_eef = np.dot(T_cam, T_cam_eef)
    print(T_eef)
    eef_pose = Pose()
    eef_pose.position.x = T_eef[0, 3]
    eef_pose.position.y = T_eef[1, 3]
    eef_pose.position.z = T_eef[2, 3]
    
    eef_r = R.from_matrix(T_eef[:3, :3])
    eef_quat = eef_r.as_quat()
    eef_pose.orientation.x = eef_quat[0]
    eef_pose.orientation.y = eef_quat[1]
    eef_pose.orientation.z = eef_quat[2]
    eef_pose.orientation.w = eef_quat[3]
    print(eef_r.as_matrix())

    

    print(eef_pose)

    # plan_and_execute(eef_pose)



if __name__ == "__main__":
    main()
