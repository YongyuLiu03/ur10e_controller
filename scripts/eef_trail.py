#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray, Marker
import moveit_commander
import sys
import numpy as np

rospy.init_node("eef_trajectory_publisher")

marker_array = MarkerArray()
max_markers = 100
threshold = 0.001

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group_names = robot.get_group_names()
move_group = moveit_commander.MoveGroupCommander(group_names[0])
marker_pub = rospy.Publisher("eef_trail", MarkerArray, queue_size=10)
prev_pose = np.array([move_group.get_current_pose().pose.position.x, move_group.get_current_pose().pose.position.y, move_group.get_current_pose().pose.position.z])

def publish_trajectory(event):
    global prev_pose
    eef_pose = move_group.get_current_pose().pose
    cur_pose = np.array([eef_pose.position.x, eef_pose.position.y, eef_pose.position.z])
    print(cur_pose, prev_pose, np.linalg.norm(cur_pose - prev_pose))

    if np.linalg.norm(cur_pose - prev_pose) > threshold:
        prev_pose = cur_pose
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose = eef_pose
        marker_array.markers.append(marker)
        if len(marker_array.markers) > max_markers:
            marker_array.markers.pop(0)
        marker_pub.publish(marker_array)

timer = rospy.Timer(rospy.Duration(1), publish_trajectory)

rospy.spin()