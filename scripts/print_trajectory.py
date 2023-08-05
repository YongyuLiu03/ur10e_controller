#!/usr/bin/env python3
# display printing trajectories 
from geometry_msgs.msg import Pose
import tf.transformations as tf
import numpy as np
import copy
import open3d as o3d


def main():

    init_pose = Pose()
    init_pose.position.x = float(0.5)
    init_pose.position.y = float(-0.1)
    init_pose.position.z = float(0.05)
    length = 0.055
    layers = 10
    fluid_width = 0.005
    surface_step = 11
    filler_step = 9

    
    cur_pose = init_pose
    waypoints = []
    for i in range(layers):
        print(f"Layer {i}: {cur_pose.position}")

        waypoints.append(copy.deepcopy(cur_pose))

        if i % 2 == 0:
            cur_pose.position.x += fluid_width
            cur_pose.position.y += fluid_width
        else:
            cur_pose.position.x -= fluid_width
            cur_pose.position.y -= fluid_width
        waypoints.append(copy.deepcopy(cur_pose))

        if i in [0, 1, layers-2, layers-1]:
            print("Printing surface")
            for j in range(surface_step):
                if i % 2 == 0:
                    if j % 2 == 0:
                        cur_pose.position.y += length - 2*fluid_width
                    else:
                        cur_pose.position.y -= length - 2*fluid_width
                else:
                    if j % 2 == 0:
                        cur_pose.position.y -= length - 2*fluid_width
                    else:
                        cur_pose.position.y += length - 2*fluid_width
                waypoints.append(copy.deepcopy(cur_pose))
                if j != surface_step-1: 
                    if i % 2 == 0:
                        cur_pose.position.x += (length-2*fluid_width)/(surface_step-1)
                    else:
                        cur_pose.position.x -= (length-2*fluid_width)/(surface_step-1)
                    waypoints.append(copy.deepcopy(cur_pose))

        else:
            print("Printing filler")            
            for j in range(filler_step):
                if i % 2 == 0:
                    if j % 2 == 0:
                        cur_pose.position.y += length - 2*fluid_width
                    else:
                        cur_pose.position.y -= length - 2*fluid_width
                    cur_pose.position.x +=  (length-2*fluid_width)/filler_step
                else:
                    if j % 2 == 0:
                        cur_pose.position.x -= length - 2*fluid_width
                    else:
                        cur_pose.position.x += length - 2*fluid_width
                    cur_pose.position.y -=  (length-2*fluid_width)/filler_step
                waypoints.append(copy.deepcopy(cur_pose))

        if i % 2 == 0:
            cur_pose.position.x += fluid_width
            cur_pose.position.y += fluid_width
        else:
            cur_pose.position.x -= fluid_width
            cur_pose.position.y -= fluid_width
        waypoints.append(copy.deepcopy(cur_pose))


        print("Printing edge")
        for j in range(4):
            if i % 2 != 0:
                if j == 0:
                    cur_pose.position.x += length
                elif j == 1:
                    cur_pose.position.y += length
                elif j == 2:
                    cur_pose.position.x -= length
                elif j == 3:
                    cur_pose.position.y -= length
            else:
                if j == 0:
                    cur_pose.position.x -= length
                elif j == 1:
                    cur_pose.position.y -= length
                elif j == 2:
                    cur_pose.position.x += length
                elif j == 3:
                    cur_pose.position.y += length                
            waypoints.append(copy.deepcopy(cur_pose))

        cur_pose.position.z += fluid_width

    points = np.empty((len(waypoints), 3))
    lines = np.empty((len(waypoints)-1, 2))
    for i in range(len(waypoints)):
        points[i][0] = waypoints[i].position.x
        points[i][1] = waypoints[i].position.y
        points[i][2] = waypoints[i].position.z
        if i != 0:
            lines[i-1][0] = i-1
            lines[i-1][1] = i
    lineset = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(points), lines=o3d.utility.Vector2iVector(lines))

    start = o3d.geometry.TriangleMesh.create_sphere(radius=0.001)
    start.translate(points[0])
    start.paint_uniform_color((1, 0, 0))
    end = o3d.geometry.TriangleMesh.create_sphere(radius=0.001)
    end.translate(points[-1])
    end.paint_uniform_color((0, 1, 0))
    o3d.visualization.draw_geometries([lineset, start, end])
    


if __name__ == "__main__":
    main()