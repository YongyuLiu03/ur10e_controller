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
init_pose.position.x = float(0.7)
init_pose.position.y = float(-0.1)
init_pose.position.z = float(0.05)
scale = 1
length = 0.1
rotate_ratio = 0.4
inter_steps = 10
inter_size = length*rotate_ratio/(2*inter_steps)



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
    rospy.init_node("2square_motion")

    robot = moveit_commander.RobotCommander()
    group_names = robot.get_group_names()
    move_group = moveit_commander.MoveGroupCommander(group_names[0])
    monitor = Monitor()
    # move_group.clear_path_constraints()

    rot_z_45 = R.from_rotvec(np.radians(45)*np.array([0, 0, 1]))

    init_r = R.from_rotvec(np.radians(45)*np.array([0, 0, 1]))
    init_quat = init_r.as_quat()
    init_pose.orientation.x = init_quat[0]
    init_pose.orientation.y = init_quat[1]
    init_pose.orientation.z = init_quat[2]
    init_pose.orientation.w = init_quat[3]

    # execute = False
    # while not execute:
    #     move_group.set_pose_target(init_pose)
    #     plan = move_group.plan()
    #     execute = "e" == input("e to excecte / other to replan ")
    # success = move_group.go(plan[0], wait=True)
    # retry = True
    # while not success and retry:
    #     plan = move_group.plan()
    #     retry = "e" == input("e to retry / other to skip ")
    #     success = move_group.go(plan[0], wait=True)

    move_group.set_pose_target(init_pose)
    plan = move_group.plan()
    success = move_group.go(plan[0], wait=True)
    while not success:
        rospy.sleep(1)
        plan = move_group.plan()
        success = move_group.go(plan[0], wait=True)
    
    z = [0, 0, 1]
    cur_pose = init_pose
    prev_r = init_r
    for i in range(1, 5):
        # if i == 1:
        #     x = [0, 1, 0]
        # elif i == 2:
        #     x = [-1, 0, 0]
        # elif i == 3:
        #     x = [0, -1, 0]
        # elif i == 4:
        #     x = tf.unit_vector([1, 0, 0])
        # y = np.cross(z, x)
        # print("y: ", y)
        # rotation_matrix = np.array([x, y, z]).transpose() 
        # print("rotation matrix: ", rotation_matrix)
        # r = R.from_matrix(rotation_matrix)
        # quat = r.as_quat()
        # print("quat: ", quat)
        # cur_pose.orientation.x = quat[0]
        # cur_pose.orientation.y = quat[1]
        # cur_pose.orientation.z = quat[2]
        # cur_pose.orientation.w = quat[3]
        # print("Orientation plan")

        waypoints = []
        cur_r = rot_z_45 * prev_r
        for t in np.linspace(0, 1, inter_steps):
            quat = (1 - t) * prev_r.as_quat() + t * cur_r.as_quat()
            quat = quat / np.linalg.norm(quat)
            pose = Pose()
            pose.position = cur_pose.position 
            if i == 1:
                pose.position.x += inter_size
            elif i == 2:
                pose.position.y += inter_size
            elif i == 3:
                pose.position.x -= inter_size
            elif i == 4:
                pose.position.y -= inter_size
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            waypoints.append(pose)
            # print("pose:", pose)
        print(waypoints[-1])
        
        # execute = False
        # while not execute:
        #     (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        #     execute = "e" == input("e to excecte / other to replan ")
        # success = move_group.execute(plan, wait=True)
        # move_group.stop()    
        # retry = True
        # while not success and retry:
        #     (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        #     retry = "e" == input("e to retry / other to skip ")
        #     success = move_group.execute(plan, wait=True)
        #     move_group.stop()

        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        success = move_group.execute(plan, wait=True)
        while not success:
            wait_until_stop(move_group)
            (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            success = move_group.execute(plan, wait=True)


        move_group.clear_pose_targets()
        prev_r = cur_r

        waypoints = []
        if i == 1:
            cur_pose.position.x += (1-rotate_ratio) * length
        elif i == 2:
            cur_pose.position.y += (1-rotate_ratio) * length
        elif i == 3:
            cur_pose.position.x -= (1-rotate_ratio) * length
        elif i == 4:
            cur_pose.position.y -= (1-rotate_ratio) * length
        cur_quat = cur_r.as_quat()
        cur_pose.orientation.x = cur_quat[0]
        cur_pose.orientation.y = cur_quat[1]
        cur_pose.orientation.z = cur_quat[2]
        cur_pose.orientation.w = cur_quat[3]
        waypoints.append(cur_pose)
        print(waypoints[-1])

        print("Cartesian path plan")
        # execute = False
        # while not execute:
        #     (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        #     execute = "e" == input("e to excecte / other to replan ")
        # success = move_group.execute(plan, wait=True)
        # # move_group.stop()    
        # retry = True
        # while not success and retry:
        #     (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        #     retry = "e" == input("e to retry / other to skip ")
        #     success = move_group.execute(plan, wait=True)
        #     move_group.stop()
        # move_group.clear_pose_targets()

        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        success = move_group.execute(plan, wait=True)
        while not success:
            wait_until_stop(move_group)
            (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            success = move_group.execute(plan, wait=True)

        waypoints = []
        cur_r = rot_z_45 * prev_r
        print("Rotating")
        print("cur_r: ", cur_r.as_matrix())
        print("prev_r: ", prev_r.as_matrix())
        
        for t in np.linspace(0, 1, inter_steps):
            quat = (1 - t) * prev_r.as_quat() + t * cur_r.as_quat()
            quat = quat / np.linalg.norm(quat)
            pose = Pose()
            pose.position = cur_pose.position 
            if i == 1:
                pose.position.x += inter_size
            elif i == 2:
                pose.position.y += inter_size
            elif i == 3:
                pose.position.x -= inter_size
            elif i == 4:
                pose.position.y -= inter_size
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            waypoints.append(pose)
        print(waypoints[-1])
        # execute = False
        # while not execute:
        #     (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        #     execute = "e" == input("e to excecte / other to replan ")
        # success = move_group.execute(plan, wait=True)
        # # move_group.stop()
        # retry = True
        # while not success and retry:
        #     (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        #     retry = "e" == input("e to retry / other to skip ")
        #     success = move_group.execute(plan, wait=True)
        #     # move_group.stop()
        # move_group.clear_pose_targets()
        # prev_r = cur_r

        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        success = move_group.execute(plan, wait=True)
        while not success:
            wait_until_stop(move_group)
            (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            success = move_group.execute(plan, wait=True)


if __name__ == "__main__":
    main()