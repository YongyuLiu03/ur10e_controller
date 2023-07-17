#!/usr/bin/env python3
import open3d as o3d
import time
import rospy
import tf2_ros
import pyrealsense2 as rs
import os
import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import copy

global tfBuffer
global listener

rospy.init_node("realtime")

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
init_pose.position.z = float(-0.3)
init_r = R.from_rotvec(np.radians(45)*np.array([0, 0, 1]))
init_quat = init_r.as_quat()
init_pose.orientation.x = init_quat[0]
init_pose.orientation.y = init_quat[1]
init_pose.orientation.z = init_quat[2]
init_pose.orientation.w = init_quat[3]
length = 0.06
fluid_width = 0.003
layers = 11
height = 0.045
bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=np.array([-2*fluid_width, -2*fluid_width, -fluid_width]), 
                                        max_bound=np.array([length+fluid_width, length+fluid_width, height+fluid_width]))

bbox.color = (0, 0, 1)


origin = np.array([init_pose.position.x, init_pose.position.y, init_pose.position.z])

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
    
    pcd_t = copy.deepcopy(pcd)
    pcd_t.transform(depth_to_color).transform(T).translate(origin)
    pcd_t = pcd_t.crop(bbox)
    return pcd_t

def get_camera_position(radius, azimuth, elevation):
    x = radius * np.sin(elevation) * np.cos(azimuth)
    y = radius * np.sin(elevation) * np.sin(azimuth)
    z = radius * np.cos(elevation)
    return [x, y, z]

def remove_hidden_points(pcd):
    pt_map_aggregated = []
    view_counter = 1
    diameter = np.linalg.norm(np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
    # print(diameter)
    radius = diameter * 100
    
    for azimuth in np.linspace(0, 2*np.pi, num=10):     # Rotate around the object
        for elevation in np.linspace(0, np.pi/2, num=5):  # From top to bottom
            camera = get_camera_position(diameter, azimuth, elevation)
            _, pt_map = pcd.hidden_point_removal(camera, radius)    
            pt_map_aggregated += pt_map
            view_counter += 1
                
    pt_map_aggregated = list(set(pt_map_aggregated))
    return pt_map_aggregated

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame())
pcd = o3d.geometry.PointCloud()

while True:
    pcd += capture_frame()  
    pt_map = remove_hidden_points(pcd)
    # pcd_visible = pcd.select_by_index(pt_map)
    # pcd_visible.paint_uniform_color([0, 0, 1])
    # pcd_hidden = pcd.select_by_index(pt_map, invert=True)
    # pcd_hidden.paint_uniform_color([1, 0, 0])    
    pcd = pcd.select_by_index(pt_map)
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    
    time.sleep(1) 



