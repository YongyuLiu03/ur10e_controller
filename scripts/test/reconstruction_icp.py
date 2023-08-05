#!/usr/bin/env python3
import numpy as np
import open3d as o3d
import os
import cv2
import copy



def main():
    count = 4
    pcd_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../pcd/")

    voxel_size = 0.0015
    radius_normal = 0.0015
    radius_feature = 0.01
    max_correspondence_distance = 0.01
    source = o3d.io.read_point_cloud(pcd_dir + "pcd_t0.ply")

    source_down = source.voxel_down_sample(voxel_size = voxel_size)
    source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    for i in range(1, count):
        target = o3d.io.read_point_cloud(pcd_dir + f'pcd_t{i}.ply')

        target_down = target.voxel_down_sample(voxel_size=voxel_size)
        target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            target_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        # result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        #     source_down, target_down, source_fpfh, target_fpfh, mutual_filter = True,
        #     max_correspondence_distance=max_correspondence_distance,
        #     estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        #     ransac_n=4,
        #     checkers=[o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
        #             o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.07)],
        #     criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
        # current_transformation = result_ransac.transformation
        current_transformation = np.identity(4)
        
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, voxel_size, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                          relative_rmse=1e-6,
                                                          max_iteration=50))
        transformation_matrix = result_icp.transformation

        source.transform(transformation_matrix)
        # source.paint_uniform_color([1, 0.706, 0])  # source is yellow
        # target.paint_uniform_color([0, 0.651, 0.929])  # target is blue
        
        o3d.visualization.draw_geometries([source, target])
        source += target
        source_down = source.voxel_down_sample(voxel_size = voxel_size)
        source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            source_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))


if __name__ == "__main__":
    main()