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


length = 0.065
fluid_width = 0.003
layers = 11
height = 0.04
pcd_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../pcd/")



box_mesh = o3d.geometry.TriangleMesh.create_box(length, length, height)

# vertices = np.asarray(box_mesh.vertices)
# triangles = np.asarray(box_mesh.triangles)
# bottom = []
# for i in range(len(triangles)):
#     if (vertices[triangles[i]][:, 2]==0).all():
#         bottom.append(i)
# triangles = np.delete(triangles, bottom, 0)
# box_mesh.triangles = o3d.utility.Vector3iVector(triangles)
# box_lineset = o3d.geometry.LineSet.create_from_triangle_mesh(box_mesh)

box_mesh.compute_vertex_normals()

o3d.io.write_triangle_mesh(pcd_dir + "box.stl", box_mesh)

new_box_mesh = o3d.io.read_triangle_mesh(pcd_dir + "box.stl")


mesh = trimesh.load_mesh(pcd_dir + "box.stl")


vertices = np.asarray(mesh.vertices)
triangles = np.asarray(mesh.faces)
bottom = []
for i in range(len(triangles)):
    if (vertices[triangles[i]][:, 2]==0).all():
        bottom.append(i)
triangles = np.delete(triangles, bottom, 0)

mesh = trimesh.Trimesh(vertices=vertices, faces=triangles)

point = [0, 0, 0.02]
normal = [0, 0, -1]

# Slice the mesh
sliced = trimesh.intersections.slice_mesh_plane(mesh, plane_normal=normal, plane_origin=point)

print(sliced)

o3d_mesh = o3d.geometry.TriangleMesh()

# Assign vertices and triangles to the open3d mesh
o3d_mesh.vertices = o3d.utility.Vector3dVector(np.array(sliced.vertices))
o3d_mesh.triangles = o3d.utility.Vector3iVector(np.array(sliced.faces))
o3d.visualization.draw_geometries([o3d_mesh])



bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-fluid_width, -fluid_width, -fluid_width),
                                        max_bound=(length+fluid_width, length+fluid_width, 0.02))
bbox.color = (0, 0, 1)


box_pcd = o3d_mesh.sample_points_uniformly(number_of_points=1000)

o3d.visualization.draw_geometries([box_pcd, bbox])
