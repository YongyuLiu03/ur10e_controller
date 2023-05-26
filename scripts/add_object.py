#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

rospy.init_node("add_object")

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

scene.remove_world_object()
print("removed previous world objects")

scene.add_box("my_box", p, size) 
print(f"add a new box of size {size} at {p.pose.position}")

rospy.sleep(2)
