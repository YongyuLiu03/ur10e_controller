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



def plan_and_execute_cartesian(waypoints, eef_step):
    fraction = 0
    while fraction < 0.95:
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, eef_step, 0.0)
        print(fraction)
    success = move_group.execute(plan, wait=True)
    print("Execution result: ", success)
    if not success :
        exit()
    return 

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("print_motion")

    global move_group

    robot = moveit_commander.RobotCommander()
    group_names = robot.get_group_names()
    move_group = moveit_commander.MoveGroupCommander(group_names[0])
    move_group.clear_path_constraints()

    # move_group.set_planning_time(20)
    # move_group.set_max_velocity_scaling_factor(0.6)
    # move_group.set_max_acceleration_scaling_factor(0.6)
    # move_group.set_num_planning_attempts(5)
    
    init_pose = Pose()
    init_pose.position.x = float(0.5)
    init_pose.position.y = float(0.0)
    init_pose.position.z = float(0.0)
    init_r = R.from_rotvec(np.radians(45)*np.array([0, 0, 1]))
    init_quat = init_r.as_quat()
    init_pose.orientation.x = init_quat[0]
    init_pose.orientation.y = init_quat[1]
    init_pose.orientation.z = init_quat[2]
    init_pose.orientation.w = init_quat[3]
    length = 0.06
    layers = 10
    fluid_width = 0.003
    surface_step = 11
    filler_step = 9
    quat_1 = R.from_rotvec(np.radians(45)*np.array([0, 0, 1])).as_quat()
    quat_2 = R.from_rotvec(np.radians(-135)*np.array([0, 0, 1])).as_quat()
    rot_90_z = R.from_rotvec(np.radians(-90)*np.array([0, 0, 1]))
    rot_90_z_counter = R.from_rotvec(np.radians(90)*np.array([0, 0, 1]))
    rot_180_z = R.from_rotvec(np.radians(180)*np.array([0, 0, 1]))
    

    cur_pose = init_pose
    cur_quat = quat_1
    cur_r = init_r
    for i in range(layers):
        print(f"Layer {i}: {cur_pose.position}")

        waypoints = []
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
            # print(cur_pose.position)
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
                # print(cur_pose.position)
                waypoints.append(copy.deepcopy(cur_pose))
                if j != surface_step-1: 
                    if i % 2 == 0:
                        cur_pose.position.x += (length-2*fluid_width)/(surface_step-1)
                    else:
                        cur_pose.position.x -= (length-2*fluid_width)/(surface_step-1)
                    waypoints.append(copy.deepcopy(cur_pose))

        else:
            print("Printing filler")            
            # print(cur_pose.position)
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
                # print(cur_pose.position)

        if i % 2 == 0:
            cur_pose.position.x += fluid_width
            cur_pose.position.y += fluid_width
        else:
            cur_pose.position.x -= fluid_width
            cur_pose.position.y -= fluid_width
        waypoints.append(copy.deepcopy(cur_pose))
        # print(cur_pose.position)
        plan_and_execute_cartesian(waypoints, fluid_width)


        print("Printing edge")
        waypoints = []
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
        plan_and_execute_cartesian(waypoints, fluid_width)

        if i % 2 == 1 and i != layers-1:
            waypoints = []
            # rot_r = rot_90_z_counter if cur_r == init_r else rot_90_z
            # for j in range(2):
            #     cur_r = rot_r * cur_r
            #     cur_quat = cur_r.as_quat()
            #     cur_pose.orientation.x = cur_quat[0]
            #     cur_pose.orientation.y = cur_quat[1]
            #     cur_pose.orientation.z = cur_quat[2]
            #     cur_pose.orientation.w = cur_quat[3]
            #     waypoints.append(copy.deepcopy(cur_pose))
            cur_r = rot_180_z * cur_r
            cur_quat = cur_r.as_quat()
            cur_pose.orientation.x = cur_quat[0]
            cur_pose.orientation.y = cur_quat[1]
            cur_pose.orientation.z = cur_quat[2]
            cur_pose.orientation.w = cur_quat[3]
            waypoints.append(copy.deepcopy(cur_pose))

            plan_and_execute_cartesian(waypoints, 1)

        cur_pose.position.z += fluid_width


if __name__ == "__main__":
    main()