#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import os

def load_point_clouds(count, voxel_size):
    pcds = []
    for i in range(count):
        pcd = o3d.io.read_point_cloud(pcd_dir + f"pcd_t{i}.pcd")
        pcd_down = pcd.voxel_down_sample(voxel_size)
        pcds.append(pcd_down)
    return pcds

def pairwise_registration(source, target):
    print("Apply point-to-point ICP")
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    transformation_coarse = icp_coarse.transformation
    information_matrix = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_coarse, transformation_coarse)
    print(icp_coarse)
    return transformation_coarse, information_matrix

def multiway_registration(pcds):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(pcds[source_id], pcds[target_id])
            pose_graph.edges.append(
                o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                        target_id,
                                                        transformation_icp,
                                                        information_icp,
                                                        uncertain=False))
    return pose_graph

# Load your point clouds
pcd_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../pcd/")

voxel_size = 0.0015
radius_normal = 0.0015
radius_feature = 0.01
max_correspondence_distance = 0.05
max_correspondence_distance_fine = 0.01
max_correspondence_distance_coarse = 0.1

pcds_down = load_point_clouds(count=8, voxel_size=voxel_size)

# Multiway registration
pose_graph = multiway_registration(pcds_down)

# Optimize the pose graph
option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.25,
        reference_node=0)
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)