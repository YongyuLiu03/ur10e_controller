#!/usr/bin/env python3
# make camera rotate around the object and capture frame from four directions, use it to verify hand-eye calibration result and printer_link position in ur_macro.xacro
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
import open3d as o3d
import os
import pyrealsense2 as rs

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("camera_reconstruct")

global move_group
global tfBuffer
global listener

robot = moveit_commander.RobotCommander()
group_names = robot.get_group_names()
move_group = moveit_commander.MoveGroupCommander(group_names[0])
move_group.clear_path_constraints()
move_group.set_planning_time(20)
move_group.set_num_planning_attempts(10)
# move_group.set_max_velocity_scaling_factor(0.1)
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

pcd_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../pcd/")
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.rgb8, 30)
profile = pipeline.start(config)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
pinhole_camera_intrinsic.set_intrinsics(
    intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy
)
o3d.io.write_pinhole_camera_intrinsic(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/camera_intrinsics.json"), pinhole_camera_intrinsic)

align_to = rs.stream.color
align = rs.align(align_to)

depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
extrinsics = depth_profile.get_extrinsics_to(color_profile)
depth_to_color = np.eye(4)
depth_to_color[:3, :3] = np.array(extrinsics.rotation).reshape(3, 3)
depth_to_color[:3, 3] = np.array(extrinsics.translation).reshape(3)
if depth_scale < 0.001:
        depth_to_color[:3, 3] *= 10
        
print(depth_scale)
print(depth_to_color)
clip_distance_in_meters = 0.5
clip_distance_in_depth_units = int(clip_distance_in_meters / depth_scale)

init_pose = Pose()
init_pose.position.x = float(0.5)
init_pose.position.y = float(0.0)
init_pose.position.z = float(-0.25)
init_r = R.from_rotvec(np.radians(0)*np.array([0, 0, 1]))
init_quat = init_r.as_quat()
init_pose.orientation.x = init_quat[0]
init_pose.orientation.y = init_quat[1]
init_pose.orientation.z = init_quat[2]
init_pose.orientation.w = init_quat[3]
length = 0.06

origin = np.array([0.5, 0.0, -0.3])

rot_z_90 = R.from_rotvec(np.radians(90)*np.array([0, 0, 1]))


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

    return transform.transform

def capture_frame():
    transform = get_transform()
    frame = align.process(pipeline.wait_for_frames())
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()
    q = transform.rotation
    r = R.from_quat([q.x, q.y, q.z, q.w])
    matrix = r.as_matrix()
    t = transform.translation
    translation = np.array([t.x, t.y, t.z])
    if depth_scale < 0.001:
        translation *= 10
    T = np.eye(4)
    T[:3, :3] = matrix
    T[:3, 3] = translation
    print(T)
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    depth_image[depth_image > clip_distance_in_depth_units] = 0
    depth_o3d = o3d.geometry.Image(depth_image)
    color_o3d = o3d.geometry.Image(color_image)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
    if depth_scale < 0.001:
        pcd.scale(0.1, (0, 0, 0))
    pcd_t = copy.deepcopy(pcd)
    pcd_t.transform(depth_to_color).transform(T).translate(-origin)
    return pcd_t

def denoise(pcd):

    cl, r_ind = pcd.remove_radius_outlier(nb_points=200, radius=0.1)
    r_inlier = pcd.select_by_index(r_ind)
    r_outlier = pcd.select_by_index(r_ind, invert=True)
    r_outlier.paint_uniform_color((1, 0, 0))
    pcd = r_inlier
    cl, s_ind = pcd.remove_statistical_outlier(nb_neighbors=500, std_ratio=2.0)
    s_inlier = pcd.select_by_index(s_ind)
    s_outlier = pcd.select_by_index(s_ind, invert=True)
    s_outlier.paint_uniform_color((1, 0, 0))

    o3d.visualization.draw_geometries([r_inlier, r_outlier])

    o3d.visualization.draw_geometries([s_inlier, s_outlier])

    return pcd

def main():
    
    cur_r = init_r
    cur_pose = init_pose
    cur_pose.position.y += 0.5*length
    plan_and_execute(cur_pose)
    time.sleep(0.5)
    combined_pcd = capture_frame()

    points = [copy.deepcopy(cur_pose.position)]
    for i in range(3):
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
        combined_pcd += denoise(capture_frame())
        points.append(copy.deepcopy(cur_pose.position))
        


    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
    sphere.paint_uniform_color((1, 0, 0))
    origin_mesh = copy.deepcopy(sphere).translate(origin)
    point_meshes = [origin_mesh]
    for i in range(len(points)):
        p = copy.deepcopy(sphere).translate([points[i].x, points[i].y, points[i].z])
        
        point_meshes.append(p)

    o3d.visualization.draw_geometries([frame, combined_pcd, *point_meshes])
    # o3d.io.write_point_cloud(pcd_dir + "pcd.pcd", combined_pcd)

if __name__ == "__main__":
    main()
