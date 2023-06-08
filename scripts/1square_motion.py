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

init_pose = Pose()
init_pose.position.x = float(0.5)
init_pose.position.y = float(-0.1)
init_pose.position.z = float(0.05)
scale = 1
length = 0.1


class Monitor:
    def __init__(self):
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.joint_velocity = None

    def joint_state_callback(self, data):
        self.joint_velocity = data.velocity

def wait_until_stop(group):
    monitor = Monitor()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if monitor.joint_velocity is not None and all(velocity < 0.01 for velocity in monitor.joint_velocity):
            break
        rate.sleep()

    return group.get_current_pose().pose

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("square_motion")

    robot = moveit_commander.RobotCommander()
    group_names = robot.get_group_names()
    move_group = moveit_commander.MoveGroupCommander(group_names[0])

    # move_group.clear_path_constraints()

    execute = False
    while not execute:
        move_group.set_pose_target(init_pose)
        plan = move_group.plan()
        execute = "e" == input("e to excecte / other to replan ")
    success = move_group.go(plan[0], wait=True)
    retry = True
    while not success and retry:
        plan = move_group.plan()
        retry = "e" == input("e to retry / other to skip ")
        success = move_group.go(plan[0], wait=True)


    z = [0, 0, 1]
    cur_pose = init_pose
    for i in range(1, 5):
        if i == 1:
            x = [0, 1, 0]
        elif i == 2:
            x = [-1, 0, 0]
        elif i == 3:
            x = [0, -1, 0]
        elif i == 4:
            x = tf.unit_vector([1, 0, 0])
        y = np.cross(z, x)
        print("y: ", y)
        rotation_matrix = np.array([x, y, z]).transpose() 
        print("rotation matrix: ", rotation_matrix)
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()
        print("quat: ", quat)
        cur_pose.orientation.x = quat[0]
        cur_pose.orientation.y = quat[1]
        cur_pose.orientation.z = quat[2]
        cur_pose.orientation.w = quat[3]
        print("Orientation plan")

        execute = False
        while not execute:
            move_group.set_pose_target(cur_pose)
            plan = move_group.plan()
            execute = "e" == input("e to excecte / other to replan ")
        success = move_group.execute(plan[1], wait=True)
        retry = True
        while not success and retry:
            plan = move_group.plan()
            retry = "e" == input("e to retry / other to skip ")
            success = move_group.execute(plan[1], wait=True)

        # move_group.stop()
        move_group.clear_pose_targets()
        if i == 0:
             continue

        for j in range(int(1/scale)):
            waypoints = []
            if i == 1:
                cur_pose.position.x += scale * length
            elif i == 2:
                cur_pose.position.y += scale * length
            elif i == 3:
                cur_pose.position.x -= scale * length
            elif i == 4:
                cur_pose.position.y -= scale * length
            waypoints.append(cur_pose)
            print("Cartesian path plan")

            execute = False
            while not execute:
                (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
                execute = "e" == input("e to excecte / other to replan ")
            success = move_group.execute(plan, wait=True)

            retry = True
            while not success and retry:
                (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
                retry = "e" == input("e to retry / other to skip ")
                success = move_group.execute(plan, wait=True)

            # move_group.stop()
            move_group.clear_pose_targets()



if __name__ == "__main__":
    main()