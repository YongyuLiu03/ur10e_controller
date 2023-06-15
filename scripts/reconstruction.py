#!/usr/bin/env python3
import rospy
import pyrealsense2 as rs
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
import open3d as o3d
import os
import cv2
import copy
from scipy.spatial.transform import Rotation as R

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
pinhole_camera_intrinsic.set_intrinsics(
    intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy
)
volume = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=4.0/512.0,
    sdf_trunc=0.04,
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

align_to = rs.stream.color
align = rs.align(align_to)

transforms = []
frames = []
count = 0

def capture_pcd_cb(transformation):

    global count    

    print(transformation.transform)

    q = transformation.transform.rotation
    r = R.from_quat([q.x, q.z, q.y, q.w])
    matrix = r.as_matrix()
    t = transformation.transform.translation
    translation = np.array([t.x, t.z, t.y])
    T = np.eye(4)
    T[:3, :3] = matrix
    T[:3, 3] = translation

    print("euler: ", r.as_euler("xyz", degrees=True))

    transforms.append(T)

    frame = align.process(pipeline.wait_for_frames())
    
    frames.append(frame)

    count += 1
    if count == 4:
        rospy.signal_shutdown("PCD collection completed")
    
    return 


    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(aligned_depth_frame.get_data())

    clip_distance_in_meters = 0.3 
    clip_distance_in_depth_units = int(clip_distance_in_meters / depth_scale)
    depth_image[depth_image > ctransformationlip_distance_in_depth_units] = 0

    color_image = cv2.normalize(color_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    color_image = color_image.astype(np.uint8)
    depth_o3d = o3d.geometry.Image(depth_image)
    color_o3d = o3d.geometry.Image(color_image)


    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image, pinhole_camera_intrinsic
    )

    pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) * depth_scale)

    volume.integrate(rgbd_image, pinhole_camera_intrinsic, np.linalg.inv(T))

    volume.integrate(pcd, pinhole_camera_intrinsic, np.linalg.inv(T))


    o3d.io.write_point_cloud(os.path.join(os.path.dirname(os.path.realpath(__file__)), f"../pcd/pcd{count}.ply"), pcd)

def main():
    rospy.init_node("reconstruction")
    sub = rospy.Subscriber("transformation", TransformStamped, capture_pcd_cb)
    rospy.spin()
    pipeline.stop()

    # mesh = volume.extract_triangle_mesh()
    # o3d.visualization.draw_geometries([mesh])
    # o3d.io.write_triangle_mesh(os.path.join(os.path.dirname(os.path.realpath(__file__)), f"../pcd/mesh.ply"), mesh)
    # o3d.io.write_voxel_grid(os.path.join(os.path.dirname(os.path.realpath(__file__)), f"../pcd/mesh.ply"), volume)


    reverse = np.array([
        [1, 0, 0, 0],  # No change along x
        [0, -1, 0, 0],  # Invert y
        [0, 0, -1, 0],  # Invert z
        [0, 0, 0, 1],  # No change
    ])
    
    combined_pcd = o3d.geometry.PointCloud()

    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    meshes = [mesh]

    for i in range(len(frames)):
        frame = frames[i]
        T = transforms[i]
        depth_frame = frame.get_depth_frame()
        color_frame = frame.get_color_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # print("depth_image:", depth_image, "color_image: ", color_image)
        depth_o3d = o3d.geometry.Image(depth_image)
        color_o3d = o3d.geometry.Image(color_image)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
        pcd.transform(np.linalg.inv(T))
        pcd.transform(reverse)
        combined_pcd += pcd
        mesh_t = copy.deepcopy(mesh).transform(np.linalg.inv(T)).transform(reverse)
        meshes.append(mesh_t)
        # o3d.visualization.draw_geometries([pcd, mesh_t])
        
        # o3d.visualization.draw_geometries([mesh, mesh_t])

    o3d.visualization.draw_geometries([combined_pcd, *meshes])

if __name__ == "__main__":
    main()