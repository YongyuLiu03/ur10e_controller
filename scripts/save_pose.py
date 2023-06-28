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
import time
import pandas as pd

def get_transform(): 
    transform = None
    rate = rospy.Rate(10.0)
    while not transform:
        try:
            transform = tfBuffer.lookup_transform('base_link', 'wrist_3_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
    return transform

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("save_pose")
    global tfBuffer
    global listener


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    datas = []

    while True:
        print(len(datas))
        exit = "e" == input("Enter to capture, e to exit ")
        if exit:
            break
        
        transform = get_transform().transform
        t = transform.translation
        q = transform.rotation
        r = R.from_quat(np.array([q.x, q.y, q.z, q.w]))
        rotvec = r.as_rotvec()
        print(rotvec)
        data = np.concatenate((np.array([t.x, t.y, t.z]), np.array(rotvec)), axis=0)
        datas.append(data)
        print(data)
        print(transform)


    df = pd.DataFrame(datas)
    df.to_csv("image/pose_data.csv",header=False)
    

if __name__ == "__main__":
    main()