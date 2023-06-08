#!/usr/bin/env python3
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys

def main():
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node("reach_pose")

    robot = moveit_commander.RobotCommander()

    group_names = robot.get_group_names()

    move_group = moveit_commander.MoveGroupCommander(group_names[0])

    current_state = robot.get_current_state()
    joint_names = current_state.joint_state.name
    joint_positions = current_state.joint_state.position

    planning_frame = move_group.get_planning_frame()

    eef_link = move_group.get_end_effector_link()

    print(f"""robot information at launching:
    group names: {group_names}
    using group {group_names[0]}
    joint names: {joint_names}
    joint positions: {joint_positions}
    planning frame: {planning_frame}
    end effector link: {eef_link}\n""")


    pose_goal = geometry_msgs.msg.Pose()

    print("current_pose: ", move_group.get_current_pose().pose)

    while 1:
        try: 
            pose_goal.position.x = float(input("Enter x position: "))
            pose_goal.position.y = float(input("Enter y position: "))
            pose_goal.position.z = float(input("Enter z position: "))
        except ValueError:
            print("Invalid type, try again")
            continue

        move_group.set_pose_target(pose_goal)

        plan = move_group.plan()

        execute = "e" == input("e to excecte / other to discard ")

        if not execute:
            continue

        success = move_group.go(plan[0], wait=True)

        move_group.stop()
        
        move_group.clear_pose_targets()
        
        print(f"Execute Result: {'Executed' if success else 'Failed'}")
        print("current_pose: ", move_group.get_current_pose().pose)


if __name__ == "__main__":
    main()