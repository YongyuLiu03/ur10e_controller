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
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import trimesh


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
        
print(depth_scale)
print(depth_to_color)
clip_distance_in_meters = 0.3
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
length = 0.065
fluid_width = 0.003
layers = 11
height = 0.044
bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-fluid_width, -fluid_width, -fluid_width), 
                                        max_bound=(length+fluid_width, length+fluid_width, height+fluid_width))

bbox.color = (0, 0, 1)

origin = np.array([init_pose.position.x, init_pose.position.y, init_pose.position.z])


height = None
new_height = False
def callback(data):
    global height
    global new_height
    height = data.data
    new_height = True
    print("new_height:", height)


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
    T = np.eye(4)
    T[:3, :3] = matrix
    T[:3, 3] = translation
    # print(T)
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
    pcd_t = pcd_t.crop(bbox)
    return T, pcd_t

def get_camera_position(radius, azimuth, elevation):
    x = radius * np.sin(elevation) * np.cos(azimuth)
    y = radius * np.sin(elevation) * np.sin(azimuth)
    z = radius * np.cos(elevation)
    return [x, y, z]

def remove_hidden_points(pcd):
    pt_map_aggregated = []
    view_counter = 1
    diameter = np.linalg.norm(np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
    radius = diameter * 100
    
    for azimuth in np.linspace(0, 2*np.pi, num=10):     
        for elevation in np.linspace(0, np.pi/2, num=5):  
            camera = get_camera_position(diameter, azimuth, elevation)
            _, pt_map = pcd.hidden_point_removal(camera, radius)    
            pt_map_aggregated += pt_map
            view_counter += 1
                
    pt_map_aggregated = list(set(pt_map_aggregated))
    return pt_map_aggregated


model = trimesh.load_mesh(pcd_dir + "box.stl")
vertices = np.asarray(model.vertices)
triangles = np.asarray(model.faces)
bottom = []
for i in range(len(triangles)):
    if (vertices[triangles[i]][:, 2]==0).all():
        bottom.append(i)
triangles = np.delete(triangles, bottom, 0)
mesh = trimesh.Trimesh(vertices=vertices, faces=triangles)
normal = [0, 0, -1]


pcd = o3d.geometry.PointCloud()
prev_T, new_pcd = capture_frame()
cl, ind = new_pcd.remove_statistical_outlier(nb_neighbors=500, std_ratio=0.4)
outliers = new_pcd.select_by_index(ind, invert=True)
outliers.paint_uniform_color([1, 0, 0])
inliers = new_pcd.select_by_index(ind)
pcd += inliers

vis = o3d.visualization.Visualizer()
vis.create_window(height=480, width=640)
vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size= 0.1))
vis.add_geometry(pcd)
vis.add_geometry(bbox)

pub = rospy.Subscriber("height", Float64, callback)


dt = 2

previous_t = time.time()
keep_running = True
update_defect = [False, False]
box_mesh = None
color_pcd = None
pcd_removed = False
while keep_running:
    
    if time.time() - previous_t > dt:
        s = time.time()
        new_T, new_pcd = capture_frame()

        if len(new_pcd.points) > 0 and not np.allclose(prev_T, new_T, rtol=1e-1):
            print("adding a frame\n", new_T)
            cl, s_ind = new_pcd.remove_statistical_outlier(nb_neighbors=500, std_ratio=0.5)
            new_pcd = new_pcd.select_by_index(s_ind)
            pcd += new_pcd
            if not pcd_removed:
                vis.update_geometry(pcd)
            else:
                vis.add_geometry(pcd, reset_bounding_box=False)
                pcd_removed = False
            print(time.time()-s)
            previous_t = time.time()
            prev_T = new_T
            update_defect[0] = True
        
        if new_height:
            if box_mesh:
                vis.remove_geometry(box_lineset, reset_bounding_box=False)
            point = [0, 0, height]
            sliced = trimesh.intersections.slice_mesh_plane(mesh, normal, point)
            box_mesh = o3d.geometry.TriangleMesh()
            box_mesh.vertices = o3d.utility.Vector3dVector(np.array(sliced.vertices))
            box_mesh.triangles = o3d.utility.Vector3iVector(np.array(sliced.faces))
            box_lineset = o3d.geometry.LineSet.create_from_triangle_mesh(box_mesh)
            vis.add_geometry(box_lineset, reset_bounding_box=False)
            new_height = False
            update_defect[1] = True
        
        if all(update_defect):
            print("update defect")
            pt_map = remove_hidden_points(pcd)
            n_pcd = pcd.select_by_index(pt_map)
            box_pcd = box_mesh.sample_points_uniformly(number_of_points=len(n_pcd.points))
            update_defect = [False, False]
            # box_bound = o3d.geometry.AxisAlignedBoundingBox(min_bound=[0, 0, 0], max_bound=[length, length, height])
            # in_box_ind = box_bound.get_point_indices_within_bounding_box(n_pcd.points)

            threshold = 0.0025
            noise_threshold = 0.005
            cmap = plt.get_cmap("seismic")
            
            # Method 1: Compare by layers

            if color_pcd:
                vis.remove_geometry(color_pcd, reset_bounding_box=False)

            color_pcd = o3d.geometry.PointCloud()
            z_range = np.arange(0, height, fluid_width)
            for z in z_range:
                slice_box = o3d.geometry.AxisAlignedBoundingBox(min_bound=[-float("inf"), -float("inf"), z], max_bound=[float("inf"), float("inf"), z+fluid_width])
                slice_pcd = n_pcd.crop(slice_box)
                if len(slice_pcd.points) <= 0:
                    continue
                in_box_ind = np.where(model.contains(np.asanyarray(slice_pcd.points)))[0]
                slice_box = trimesh.intersections.slice_mesh_plane(trimesh.intersections.slice_mesh_plane(mesh, [0, 0, 1], [0, 0, z]), [0, 0, -1], [0, 0, z+fluid_width])
                slice_mesh = o3d.geometry.TriangleMesh()
                slice_mesh.vertices = o3d.utility.Vector3dVector(np.array(slice_box.vertices))
                slice_mesh.triangles = o3d.utility.Vector3iVector(np.array(slice_box.faces))
                slice_box_pcd = slice_mesh.sample_points_uniformly(number_of_points=len(slice_pcd.points))
                distances = slice_box_pcd.compute_point_cloud_distance(slice_pcd)
                distances = np.asanyarray(distances)
                distances[in_box_ind] *= -1
                outliers = (np.abs(distances) > threshold).nonzero()
                noise = (np.abs(distances) > noise_threshold).nonzero()
                distances[noise] = 0.0
                norm_distances = (distances - distances.min()) / (distances.max() - distances.min())
                cmap_colors = cmap(norm_distances)[:, :3]
                colors = np.asarray(slice_pcd.colors)
                colors[outliers] = cmap_colors[outliers]
                slice_pcd.colors = o3d.utility.Vector3dVector(colors)
                color_pcd += slice_pcd

            vis.add_geometry(color_pcd, reset_bounding_box=False)

            # Method 2: Compare as a whole
            # distances = box_pcd.compute_point_cloud_distance(n_pcd)
            # distances = np.asanyarray(distances)
            # distances[in_box_ind] *= -1
            # outliers = (np.abs(distances) > threshold).nonzero()
            # noise = (np.abs(distances) > noise_threshold).nonzero()
            # # distances[noise] = 0.0
            # norm_distances = (distances - distances.min()) / (distances.max() - distances.min())

            # cmap = plt.get_cmap("seismic")
            # cmap_colors = cmap(norm_distances)[:, :3]
            # colors = np.asarray(n_pcd.colors)
            # colors[outliers] = cmap_colors[outliers]
            # n_pcd.colors = o3d.utility.Vector3dVector(colors)
            # vis.add_geometry(n_pcd, reset_bounding_box=False)

            vis.remove_geometry(pcd, reset_bounding_box=False)
            pcd_removed = True
            pcd = n_pcd
            print("done updating defect")



            
    keep_running = vis.poll_events()
    vis.update_renderer()
    
vis.destroy_window()

