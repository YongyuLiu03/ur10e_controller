#!/usr/bin/env python3
# compute and store plans to print a cubic, enter in terminal to execute plans
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point
import tf.transformations as tf
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import copy
import os
import pyrealsense2 as rs
import open3d as o3d
import tf2_ros
import serial
import time
from moveit_msgs.msg import RobotState, JointConstraint, Constraints, PositionConstraint
from std_msgs.msg import Float64
import pickle

# arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)
time.sleep(2)
printhead_delay = 800

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("print_reconstruct")
pub = rospy.Publisher('height', Float64, queue_size=10)

robot = moveit_commander.RobotCommander()
group_names = robot.get_group_names()
move_group = moveit_commander.MoveGroupCommander(group_names[0])
move_group.clear_path_constraints()

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

constraint = JointConstraint()
constraint.joint_name = "wrist_2_joint"
constraint.position = -3.14
constraint.tolerance_above = 2
constraint.tolerance_below = 2
constraint.weight = 1.0

constraints = Constraints()
constraints.joint_constraints.append(constraint)

move_group.set_path_constraints(constraints)


init_pose = Pose()
init_pose.position.x = float(0.5)
init_pose.position.y = float(0.0)
init_pose.position.z = float(-0.3)
init_r = R.from_rotvec(np.radians(45)*np.array([0, 0, 1]))
init_quat = init_r.as_quat()
init_pose.orientation.x = init_quat[0]
init_pose.orientation.y = init_quat[1]
init_pose.orientation.z = init_quat[2]
init_pose.orientation.w = init_quat[3]
length = 0.06
layers = 11
fluid_width = 0.002
surface_step = 11
filler_step = 9
rot_180_z = R.from_rotvec(np.radians(180)*np.array([0, 0, 1]))
rot_90_z = R.from_rotvec(np.radians(90)*np.array([0, 0, 1]))
rot_90_z_inv = R.from_rotvec(np.radians(-90)*np.array([0, 0, 1]))
rot_175_z = R.from_rotvec(np.radians(175)*np.array([0, 0, 1]))
rot_175_z_inv = R.from_rotvec(np.radians(-175)*np.array([0, 0, 1]))


origin = np.array([init_pose.position.x, init_pose.position.y, init_pose.position.z])
plans = []
rotate_ind = []


def plan_and_execute_cartesian(waypoints, eef_step):
    fraction = 0
    while fraction < 0.95:
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, eef_step, 0.0)
        print(fraction)
    success = move_group.execute(plan, wait=True)
    print("Execution result: ", success)
    if not success :
        # arduino.write(bytes('-1', 'utf-8'))
        # time.sleep(0.5)
        # arduino.write(bytes('-1', 'utf-8'))
        exit()
    return 

def plan_cartesian(waypoints, eef_step):
    if plans:
        last_point = plans[-1].joint_trajectory.points[-1]
        robot_state = RobotState()
        robot_state.joint_state.name = move_group.get_active_joints()
        robot_state.joint_state.position = last_point.positions
        move_group.set_start_state(robot_state)
    fraction = 0
    while fraction < 0.95:
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, eef_step, 0.0)
        if eef_step > fluid_width:
            trajectory = plan.joint_trajectory            
            for j, point in enumerate(trajectory.points):
                print(f"Point #{j}: {trajectory.joint_names[4]}: { point.positions[4]}")
        print(fraction)
    return plan 


def main():

    plan_and_execute_cartesian([init_pose], 10*fluid_width)
    cur_pose = init_pose
    cur_r = init_r
    for i in range(layers):
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
        plans.append(plan_cartesian(waypoints, fluid_width))

        if i % 3 == 1:
            print("Rotating")
            if len(rotate_ind) % 2 == 0:
                rot = rot_175_z
            else:
                rot = rot_175_z_inv
            waypoints = []
            cur_r = rot * cur_r
            cur_quat = cur_r.as_quat()
            cur_pose.orientation.x = cur_quat[0]
            cur_pose.orientation.y = cur_quat[1]
            cur_pose.orientation.z = cur_quat[2]
            cur_pose.orientation.w = cur_quat[3]
            waypoints.append(copy.deepcopy(cur_pose))
            plans.append(plan_cartesian(waypoints, 0.1))
            rotate_ind.append(len(plans)-1)

        cur_pose.position.z += fluid_width

    plans_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../plans/", "plan.pickle")

    print(rotate_ind)
    with open(plans_dir, "wb") as file:
        pickle.dump([plans, rotate_ind], file)

    print("computed plans saved to", plans_dir)
    
    # return
    # with open(plans_dir, 'rb') as file:
    #     plans, rotate_ind = pickle.load(file)

    # print(loaded_plans[1])
    # print(len(loaded_plans))
    # print(loaded_rotate_ind)


    # arduino.write(bytes(printhead_delay, 'utf-8'))
    resume = input("Enter to start")
    height = 0
    for i in range(len(plans)):
        if i in rotate_ind:
            # arduino.write(bytes('-1', 'utf-8'))
            plan = plans[i]
            trajectory = plan.joint_trajectory
            for j, point in enumerate(trajectory.points):
                print(f"Point #{j}: {trajectory.joint_names[4]}: { point.positions[4]}")
        
            success = move_group.execute(plans[i], wait=True)
            print("Execution result: ", success)
            if not success :
                break
            # arduino.write(bytes(printhead_delay, 'utf-8'))
        else:
            success = move_group.execute(plans[i], wait=True)
            print("Execution result: ", success)
            if not success :
                break
            height += fluid_width
            pub.publish(height)

    # arduino.write(bytes('-1', 'utf-8'))
    # time.sleep(0.5)
    # arduino.write(bytes('-1', 'utf-8'))
    

if __name__ == "__main__":
    main()