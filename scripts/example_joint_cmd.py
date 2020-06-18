#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from std_msgs.msg import String

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint',
               'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

client = None


def process(msg):
    global curr_angle, count
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    joint_angles = msg.data[1:-1].split(",")
    joint_angles = [float(i) for i in joint_angles]

    g.trajectory.points = [
        JointTrajectoryPoint(positions=joint_angles, velocities=[0]*6, time_from_start=rospy.Duration(1.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise


if __name__ == "__main__":
    try:
        rospy.init_node("test_move_node", disable_signals=True)
        rospy.loginfo("Starting test_move_node.")
        client = actionlib.SimpleActionClient(
            'scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for server...")
        client.wait_for_server()
        rospy.loginfo("Connected to server")

        subscriber = rospy.Subscriber(
            "target_angles", String, process, queue_size=1)

        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            rate.sleep()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
