#!/usr/bin/env python3
import rospy
import pyrealsense2 as rs
import numpy as np
from geometry_msgs.msg import TransformStamped
import open3d as o3d
import os
import copy
from scipy.spatial.transform import Rotation as R
import tf.transformations as tf
import tf2_ros
import matplotlib.pyplot as plt

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

transforms = []
frames = []
count = 0

clip_distance_in_meters = 0.5
clip_distance_in_depth_units = int(clip_distance_in_meters / depth_scale)

def capture_pcd_cb(transformation):

    global count    


    q = transformation.transform.rotation
    r = R.from_quat([q.x, q.y, q.z, q.w])
    matrix = r.as_matrix()
    t = transformation.transform.translation
    translation = np.array([t.x, t.y, t.z])
    if depth_scale < 0.001:
        translation *= 10
    T = np.eye(4)
    T[:3, :3] = matrix
    T[:3, 3] = translation

    print(T)
    print("euler:", r.as_euler("yzx", degrees=True))
    transforms.append(T)
    
    frame = align.process(pipeline.wait_for_frames())
    frames.append(frame)

    count += 1
    if count == 4:
        rospy.signal_shutdown("PCD collection completed")
    return 

def main():
    rospy.init_node("reconstruction")

    sub = rospy.Subscriber("transformation", TransformStamped, capture_pcd_cb)
    rospy.spin()
    pipeline.stop()

    combined_pcd = o3d.geometry.PointCloud()

    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    meshes = [mesh]

    for i in range(0, len(frames)):
        frame = frames[i]
        T = transforms[i]
        depth_frame = frame.get_depth_frame()
        color_frame = frame.get_color_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_image[depth_image > clip_distance_in_depth_units] = 0
        depth_o3d = o3d.geometry.Image(depth_image)
        color_o3d = o3d.geometry.Image(color_image)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
        pcd_t = copy.deepcopy(pcd)
        # pcd_t.rotate(T[:3, :3])
        # pcd_t.translate(T[:3, 3])
        pcd_t.transform(depth_to_color).transform(T)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        frame_pcd = frame.sample_points_uniformly(number_of_points=1000)
        
        pcd_t.points = o3d.utility.Vector3dVector(np.vstack((np.asarray(pcd_t.points), np.asarray(frame_pcd.points))))

        pcd_t.colors = o3d.utility.Vector3dVector(np.vstack((np.asarray(pcd_t.colors), np.asarray(frame_pcd.colors))))


        combined_pcd += pcd_t
        mesh_t = copy.deepcopy(mesh).transform(T)
        meshes.append(mesh_t)

        # color = [0.5, 0, 0]  # RGB, each component should be a float in [0.0, 1.0]
        # Assign the color to all points
        # pcd_t.colors = o3d.utility.Vector3dVector([color for _ in range(len(pcd_t.points))])

        # o3d.visualization.draw_geometries([pcd_t, pcd, mesh, mesh_t])
        # o3d.io.write_point_cloud(pcd_dir + f"pcd{i}.ply", pcd)
        # o3d.io.write_point_cloud(pcd_dir + f"pcd_t{i}.ply", pcd_t)
    
    o3d.visualization.draw_geometries([combined_pcd])
    # combined_pcd_down1 = combined_pcd.voxel_down_sample(0.0005)
    # o3d.visualization.draw_geometries([combined_pcd_down1])
    # o3d.io.write_point_cloud(pcd_dir + "combine_pcd.pcd", combined_pcd)
    # combined_pcd.estimate_normals()
    # o3d.visualization.draw_geometries([combined_pcd], point_show_normal=True)

    # with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    #     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    #         combined_pcd, depth=9)
    # o3d.visualization.draw_geometries([mesh])
    
    # densities = np.asarray(densities)
    # density_colors = plt.get_cmap('plasma')(
    #     (densities - densities.min()) / (densities.max() - densities.min()))
    # density_colors = density_colors[:, :3]
    # density_mesh = o3d.geometry.TriangleMesh()
    # density_mesh.vertices = mesh.vertices
    # density_mesh.triangles = mesh.triangles
    # density_mesh.triangle_normals = mesh.triangle_normals
    # density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
    # o3d.visualization.draw_geometries([density_mesh])
    # vertices_to_remove = densities < np.quantile(densities, 0.01)
    # mesh.remove_vertices_by_mask(vertices_to_remove)
    # o3d.visualization.draw_geometries([mesh])
                                  



if __name__ == "__main__":
    main()