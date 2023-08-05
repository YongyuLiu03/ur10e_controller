#!/usr/bin/env python3
# print with calculated paths stored in ur10e_controller/plans
# will initialize position of robot first, then enter to exxecute plans
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import tf.transformations as tf
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import serial
import time
from std_msgs.msg import Float64
import pickle

# uncomment arduino part to control printhead motor in this program
# arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)
time.sleep(2)
# modify this
printhead_delay = 800

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("direct_print")
pub = rospy.Publisher('height', Float64, queue_size=10)

robot = moveit_commander.RobotCommander()
group_names = robot.get_group_names()
move_group = moveit_commander.MoveGroupCommander(group_names[0])
move_group.clear_path_constraints()

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
fluid_width = 0.002


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


def main():

    plan_and_execute_cartesian([init_pose], 10*fluid_width)

    # modify this
    plans_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../plans/", "plan.pickle")
    with open(plans_dir, 'rb') as file:
        plans, rotate_ind = pickle.load(file)

    wawit = input("enter to start")
    # arduino.write(bytes(printhead_delay, 'utf-8'))
    height = 0
    for i in range(len(plans)):
        if i in rotate_ind:
            # arduino.write(bytes('-1', 'utf-8'))
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