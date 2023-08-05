#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

rospy.init_node("add_collision")

scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

rospy.sleep(2)

p = PoseStamped()
p.header.frame_id = robot.get_planning_frame() 
p.pose.position.x = 0.6
p.pose.position.y = 0.6
p.pose.position.z = 0.1
p.pose.orientation.w = 1.0
size = (0.2, 0.2, 0.2)

floor = PoseStamped()
floor.header.frame_id = robot.get_planning_frame()
floor.pose.position.z = 0
floor.pose.position.x = 1
floor.pose.position.y = 1
floor.pose.orientation.w = 1.0
floor_n = (0, 0, 1)

wall_along_x = PoseStamped()
wall_along_x.header.frame_id = robot.get_planning_frame()
wall_along_x.pose.position.z = 1
wall_along_x.pose.position.x = 1
wall_along_x.pose.position.y = -0.3
wall_along_x.pose.orientation.w = 1.0
wall_along_x_n = (0, 1, 0)

wall_along_y = PoseStamped()
wall_along_y.header.frame_id = robot.get_planning_frame()
wall_along_y.pose.position.z = 1
wall_along_y.pose.position.x = -0.3
wall_along_y.pose.position.y = 1
wall_along_y.pose.orientation.w = 1.0
wall_along_y_n = (1, 0, 0)

plane_size = (3, 3, 0.01)

scene.remove_world_object()
print("removed previous world objects")


scene.add_box("floor", floor, plane_size)
scene.add_box("wall_along_x", wall_along_x, (3, 0.01, 3))
scene.add_box("wall_along_y", wall_along_y, (0.01, 3, 3))
# scene.add_box("my_box", p, size) 
# print(f"add a new box of size {size} at {p.pose.position}")
print("added collision objects")

rospy.sleep(2)
