#!/usr/bin/env python3
import rospy
import pyrealsense2 as rs
import numpy as np
from geometry_msgs.msg import PoseStamped
import open3d as o3d


def capture_pcd_cb(camera_pose):
    print(camera_pose)
    R = o3d.geometry.get_ratation_matrix_from_quaternion([camera_pose.pose.orientaion.x, camera_pose.pose.orientaion.y, camera_pose.pose.orientaion.z, camera_pose.pose.orientaion.w])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [camera_pose.pose.position.x, camera_pose.pose.position.y, camera_pose.pose.position.z]
    depth = None
    while not depth:
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        depth_image = np.asanyarray(depth.get_data())
        depth_o3d = o3d.geometry.Image(depth_image)
        pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_o3d, o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
        pcd.transform(T)
        if not depth: continue
    
    combined_pcd += pcd
    count += 1
    if count == 4:
        rospy.signal_shutdown("PCD collection completed")

def main():
    rospy.init_node("reconstruction")

    sub = rospy.Subscriber("camera_pose", PoseStamped, capture_pcd_cb)

    global pipeline
    global count
    global combined_pcd
    count = 0
    combined_pcd = o3d.geometry.PointCloud()

    pipeline = rs.pipeline()
    pipeline.start()

    rospy.spin()
    pipeline.stop()
    o3d.io.write_point_cloud("~/catkin_ws/files/combined_pcd.pcd", combined_pcd)
    

if __name__ == "__main__":
    main()