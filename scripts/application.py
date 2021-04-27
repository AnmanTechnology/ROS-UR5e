#!/usr/bin/env python
import math
import sys

import geometry_msgs.msg
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
import rospy

class RobotControlNode:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("robot_control_node")
        rospy.loginfo("Starting RobotControlNode as robot_control_node.")
        
        self.robot_A = moveit_commander.RobotCommander(robot_description="urA/robot_description",ns="urA")
        self.scene_A = moveit_commander.PlanningSceneInterface(ns='urA')
        self.move_group_A = moveit_commander.MoveGroupCommander("manipulator",robot_description="urA/robot_description",ns="urA")
        self.display_trajectory_publisher_A = rospy.Publisher('urA/move_group/display_planned_path',DisplayTrajectory,queue_size=20)

        self.robot_B = moveit_commander.RobotCommander(robot_description="urB/robot_description",ns="urB")
        self.scene_B = moveit_commander.PlanningSceneInterface(ns='urB')
        self.move_group_B = moveit_commander.MoveGroupCommander("manipulator",robot_description="urB/robot_description",ns="urB")
        self.display_trajectory_publisher_B = rospy.Publisher('urB/move_group/display_planned_path',DisplayTrajectory,queue_size=20)

        # planning_frame = move_group.get_planning_frame()
        # rospy.loginfo("============ Reference frame: {}".format(planning_frame))

        # eef_link = move_group.get_end_effector_link()
        # print "============ End effector link: %s" % move_group.get_end_effector_link()
        # print "============ Available Planning Groups:", robot.get_group_names()
        # print "============ Printing robot state", robot.get_current_state() 
    

        
        joint_goal = self.move_group_A.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -math.pi/2
        joint_goal[2] = 0
        joint_goal[3] = -math.pi/2
        joint_goal[4] = 0
        joint_goal[5] = 0
        self.move_group_A.go(joint_goal, wait=True)
        

        

        # move_group.execute(plan, wait=True)
        # Calling `stop()` ensures that there is no residual movement
        # move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        # move_group.clear_pose_targets()
    
    def plan_to_pose_A(self, pose):
        return self.move_group_A.plan(pose)
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.x = 0.124
        # pose_goal.orientation.y = -0.427
        # pose_goal.orientation.z = 0.625
        # pose_goal.orientation.w = 0.642
        # pose_goal.position.x = 0.559
        # pose_goal.position.y = 0.355
        # pose_goal.position.z = 1.732
    
    def plan_to_joint_A(self, joint):
        # joint_goal = move_group.get_current_joint_values()
        # joint_goal[0] = 0
        # joint_goal[1] = -math.pi/2
        # joint_goal[2] = 0
        # joint_goal[3] = -math.pi/2
        # joint_goal[4] = 0
        # joint_goal[5] = 0
        # move_group.go(joint_goal, wait=True)
        pass
    
    def publish_plan_A(self, plan):
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot_A.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher_A.publish(display_trajectory)
    
    def publish_plan_B(self, plan):
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot_B.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher_B.publish(display_trajectory)

    def test_run(self):
        pose_goal = self.move_group_A.get_current_pose()
        pose_goal.pose.orientation.x = 0
        pose_goal.pose.orientation.y = 0
        pose_goal.pose.orientation.z = 0
        pose_goal.pose.orientation.w = 1.0
        # pose_goal.pose.position.z -= 0.1
        # pose_goal.pose.position.x += 0.1
        planA = self.move_group_A.plan(pose_goal)
        self.publish_plan_A(planA)
        # pose_goal = self.move_group_B.get_current_pose()
        # pose_goal.pose.position.z -= 0.1
        # planB = self.move_group_B.plan(pose_goal)
        # self.publish_plan_B(planB)

        self.move_group_A.execute(planA, wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group_A.stop()
    
if __name__ == "__main__":
    robot_control_node = RobotControlNode()
    # robot_control_node.test_run()
    rospy.spin()
