#!/usr/bin/env python3
import sys
import math
import numpy as np
import copy
import os
import open3d as o3d
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pandas as pd

def get_camera_position(radius, azimuth, elevation):
    x = radius * np.sin(elevation) * np.cos(azimuth)
    y = radius * np.sin(elevation) * np.sin(azimuth)
    z = radius * np.cos(elevation)
    return [x, y, z]

def remove_hidden_points(pcd):
    pt_map_aggregated = []
    view_counter = 1
    diameter = np.linalg.norm(np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
    print(diameter)
    radius = diameter * 100
    
    for azimuth in np.linspace(0, 2*np.pi, num=10):     # Rotate around the object
        for elevation in np.linspace(0, np.pi/2, num=5):  # From top to bottom
            camera = get_camera_position(diameter, azimuth, elevation)
            _, pt_map = pcd.hidden_point_removal(camera, radius)    
            pt_map_aggregated += pt_map
            view_counter += 1
                
    pt_map_aggregated = list(set(pt_map_aggregated))
    return pt_map_aggregated


pcd_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../pcd/")
pcd = o3d.io.read_point_cloud(pcd_dir + "pcd.pcd")
# pcd.scale(0.1, (0, 0, 0))

length = 0.065
layers = 11
fluid_width = 0.003
height = 0.045

frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
frame_pcd = frame.sample_points_uniformly(number_of_points=1000)

box_l = np.array([0, 0, 0])
box_h = np.array([length, length, height])
box_mesh = o3d.geometry.TriangleMesh.create_box(*(box_h-box_l))
box_mesh.translate(box_l)
vertices = np.asarray(box_mesh.vertices)
triangles = np.asarray(box_mesh.triangles)
bottom = []
for i in range(len(triangles)):
    if (vertices[triangles[i]][:, 2]==0).all():
        bottom.append(i)
triangles = np.delete(triangles, bottom, 0)
box_mesh.triangles = o3d.utility.Vector3iVector(triangles)


bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=np.array([-2*fluid_width, -2*fluid_width, -fluid_width]), 
                                           max_bound=np.array([length+fluid_width, length+fluid_width, height+fluid_width]))
bbox.color = (0, 0, 1)

o3d.visualization.draw_geometries([pcd, bbox, frame_pcd, box_mesh])
pcd = pcd.crop(bbox)

# pt_map = remove_hidden_points(pcd)
# pcd_visible = pcd.select_by_index(pt_map)
# pcd_visible.paint_uniform_color([0, 0, 1])
# pcd_hidden = pcd.select_by_index(pt_map, invert=True)
# pcd_hidden.paint_uniform_color([1, 0, 0])
# o3d.visualization.draw_geometries([pcd_hidden, pcd_visible, bbox])
# pcd = pcd.select_by_index(pt_map)

# cl, ind = pcd.remove_radius_outlier(nb_points=2000, radius=0.04)
# inlier_pcd = pcd.select_by_index(ind)
# outlier_pcd = pcd.select_by_index(ind, invert=True)
# outlier_pcd.paint_uniform_color([1, 0, 0])
# o3d.visualization.draw_geometries([inlier_pcd, outlier_pcd])
# pcd = inlier_pcd

box_pcd = box_mesh.sample_points_uniformly(number_of_points=len(pcd.points))
o3d.visualization.draw_geometries([box_pcd, pcd])

# reg_p2p = o3d.pipelines.registration.registration_icp(
#     box_pcd, pcd, threshold=0.02)

box_bound = o3d.geometry.AxisAlignedBoundingBox(min_bound=box_l, max_bound=box_h)
in_box_ind = box_bound.get_point_indices_within_bounding_box(pcd.points)


distances = box_pcd.compute_point_cloud_distance(pcd)
distances = np.asanyarray(distances)
distances[in_box_ind] *= -1

threshold = 0.003
outliers = (np.abs(distances) > threshold).nonzero()
print(distances.min())
print(distances.max())

noise_threshold = 0.005
noise = (np.abs(distances) > noise_threshold).nonzero()
distances[noise] = 0.0

norm_distances = (distances - distances.min()) / (distances.max() - distances.min())
cmap = plt.get_cmap("seismic")
cmap_colors = cmap(norm_distances)[:, :3]

# df = pd.DataFrame({"distances": norm_distances}) # transform to a dataframe
# # Some graphs
# ax1 = df.boxplot(return_type="axes") # BOXPLOT
# ax2 = df.plot(kind="hist", alpha=0.5, bins = 1000) # HISTOGRAM
# ax3 = df.plot(kind="line") # SERIE
# plt.show()

colors = np.asarray(pcd.colors)
colors[outliers] = cmap_colors[outliers]
# colors = cmap_colors

pcd.colors = o3d.utility.Vector3dVector(colors)

o3d.visualization.draw_geometries([pcd])


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

# o3d.visualization.draw_geometries([pcd])
# o3d.io.write_point_cloud(pcd_dir+"pcd.pcd", pcd)
# pcd = o3d.io.read_point_cloud(pcd_dir + "pcd.pcd")


