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



def plan_and_execute_cartesian(waypoints):
    execute = False
    fraction = 0
    while not execute:
        while fraction < 0.95:
            (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.005, 0.0)
            print(fraction)
        execute = "e" == input("e to excecte / other to replan ")
    success = move_group.execute(plan, wait=True)
    retry = True
    while not success and retry:
        retry = "e" == input("e to retry / other to skip ")
        success = move_group.execute(plan, wait=True)
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
    length = 0.1
    layers = 10
    fluid_width = 0.005
    surface_step = 11
    filler_step = 9

    cur_pose = init_pose
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
        plan_and_execute_cartesian(waypoints)


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
            # print(cur_pose.position)
        plan_and_execute_cartesian(waypoints)

        cur_pose.position.z += fluid_width


if __name__ == "__main__":
    main()