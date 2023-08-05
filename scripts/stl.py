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


length = 0.065
fluid_width = 0.003
layers = 11
height = 0.044
pcd_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../pcd/")


box_mesh = o3d.geometry.TriangleMesh.create_box(length, length, height)

vertices = np.asarray(box_mesh.vertices)
triangles = np.asarray(box_mesh.triangles)
bottom = []
for i in range(len(triangles)):
    if (vertices[triangles[i]][:, 2]==0).all():
        bottom.append(i)
triangles = np.delete(triangles, bottom, 0)
box_mesh.triangles = o3d.utility.Vector3iVector(triangles)
box_lineset = o3d.geometry.LineSet.create_from_triangle_mesh(box_mesh)

box_mesh.compute_vertex_normals()

o3d.io.write_triangle_mesh(pcd_dir + "box.stl", box_mesh)

new_box_mesh = o3d.io.read_triangle_mesh(pcd_dir + "box.stl")

o3d.visualization.draw_geometries([new_box_mesh])

box_pcd = box_mesh.sample_points_uniformly(number_of_points=1000)

o3d.visualization.draw_geometries([box_pcd])
