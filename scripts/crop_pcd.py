#!/usr/bin/env python3
import sys
import math
import numpy as np
import copy
import os
import open3d as o3d
import matplotlib.pyplot as plt

pcd_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../pcd/")

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
    
    for azimuth in np.linspace(0, 2*np.pi, num=10):     # Rotate around the object
        for elevation in np.linspace(0, np.pi/2, num=5):  # From top to bottom
            camera = get_camera_position(diameter, azimuth, elevation)
            _, pt_map = pcd.hidden_point_removal(camera, radius)    
            pt_map_aggregated += pt_map
            view_counter += 1
                
    pt_map_aggregated = list(set(pt_map_aggregated))
    return pt_map_aggregated


cameras = []
for azimuth in np.linspace(0, 2*np.pi, num=10):     # Rotate around the object
    for elevation in np.linspace(0, np.pi/2, num=5):  # From top to bottom
        camera = get_camera_position(1, azimuth, elevation)
        view_point = o3d.geometry.TriangleMesh.create_sphere(radius=0.02) # Change the radius according to your point cloud scale
        view_point.paint_uniform_color([0, 1, 0])  # Color the sphere with Red color
        view_point.translate(camera)
        cameras.append(view_point)
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
frame_pcd = frame.sample_points_uniformly(number_of_points=1000)
# o3d.visualization.draw_geometries([frame_pcd, *cameras])

pcd = o3d.io.read_point_cloud(pcd_dir + "combined_pcd.pcd")
# pcd = pcd.voxel_down_sample(voxel_size=0.002)

aabb = pcd.get_axis_aligned_bounding_box()
aabb.color = (1, 0, 0)
obb = pcd.get_oriented_bounding_box()
obb.color = (0, 1, 0)
# o3d.visualization.draw_geometries([pcd, aabb, obb])
print(aabb.max_bound[2])
print(obb)
                                  

# o3d.visualization.draw_geometries([pcd, *cameras])
box = o3d.geometry.AxisAlignedBoundingBox(min_bound=np.array([-0.1, -0.1, -0.01]), 
                                           max_bound=np.array([0.14, 0.1, 0.05]))
box.color = (0, 1, 0)


pcd = pcd.crop(box)
pt_map = remove_hidden_points(pcd)
pcd_visible = pcd.select_by_index(pt_map)
pcd_visible.paint_uniform_color([0, 0, 1])
pcd_hidden = pcd.select_by_index(pt_map, invert=True)
pcd_hidden.paint_uniform_color([1, 0, 0])
o3d.visualization.draw_geometries([pcd_hidden, pcd_visible, box])

pcd = pcd.select_by_index(pt_map)


# o3d.visualization.draw_geometries([pcd, box, frame_pcd])



# plane_model, inliers = pcd.segment_plane(distance_threshold=0.001,
#                                         ransac_n=4,
#                                         num_iterations=1000)
# [a, b, c, d] = plane_model
# print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

# inlier_cloud = pcd.select_by_index(inliers)
# inlier_cloud.paint_uniform_color([1.0, 0, 0])
# outlier_cloud = pcd.select_by_index(inliers, invert=True)
# o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

# pcd = outlier_cloud



cl, ind = pcd.remove_radius_outlier(nb_points=2000, radius=0.04)
inlier_pcd = pcd.select_by_index(ind)
outlier_pcd = pcd.select_by_index(ind, invert=True)
outlier_pcd.paint_uniform_color([1, 0, 0])
o3d.visualization.draw_geometries([inlier_pcd, outlier_pcd])
pcd = inlier_pcd



o3d.visualization.draw_geometries([pcd])
o3d.io.write_point_cloud(pcd_dir+"pcd.pcd", pcd)