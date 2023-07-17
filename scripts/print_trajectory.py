#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import DisplayTrajectory, JointConstraint, Constraints
import tf.transformations as tf
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import copy
import open3d as o3d


def plan_and_execute_cartesian(waypoints):
    execute = False
    fraction = 0
    # while not execute:
    while fraction < 0.95:
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.005, 0.0)
        print(fraction)
        # execute = "e" == input("e to excecte / other to replan ")
    success = move_group.execute(plan, wait=True)
    wait_until_stop()
    # retry = True
    # while not success and retry:
    #     # retry = "e" == input("e to retry / other to skip ")
    #     success = move_group.execute(plan, wait=True)
    #     wait_until_stop()
    
    return 

class Monitor:
    def __init__(self):
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.joint_velocity = None

    def joint_state_callback(self, data):
        self.joint_velocity = data.velocity

def wait_until_stop():
    monitor = Monitor()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if monitor.joint_velocity is not None and all(velocity < 0.01 for velocity in monitor.joint_velocity):
            break
        rate.sleep()

    return move_group.get_current_state()

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("print_motion")

    global move_group

    robot = moveit_commander.RobotCommander()
    group_names = robot.get_group_names()
    move_group = moveit_commander.MoveGroupCommander(group_names[0])
    move_group.clear_path_constraints()

    move_group.set_planning_time(20)
    move_group.set_max_velocity_scaling_factor(0.6)
    move_group.set_max_acceleration_scaling_factor(0.6)
    move_group.set_num_planning_attempts(5)
    
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