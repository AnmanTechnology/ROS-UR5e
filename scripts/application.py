#!/usr/bin/env python
import rospy
import sys
import moveit_commander


class RobotControlNode:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("robot_control_node")
        rospy.loginfo("Starting RobotControlNode as robot_control_node.")
        robot = moveit_commander.RobotCommander()


if __name__ == "__main__":
    robot_control_node = RobotControlNode()
    rospy.spin()
