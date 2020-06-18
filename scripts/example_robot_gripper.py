#!/usr/bin/env python
import rospy
from ur_msgs.srv import SetIO, SetIORequest

if __name__ == "__main__":
    rospy.init_node("example_robot_gripper")
    rospy.loginfo("Starting example_robot_gripper.")
    rospy.wait_for_service('ur_hardware_interface/set_io')
    rospy.loginfo("Waiting for server...")
    set_io = rospy.ServiceProxy('ur_hardware_interface/set_io', SetIO)
    rospy.loginfo("Connected to server")

    while not rospy.is_shutdown():
        state = SetIORequest.STATE_ON
        resp = set_io(SetIORequest.FUN_SET_DIGITAL_OUT, 3, state)
        rospy.sleep(1)
        state = SetIORequest.STATE_OFF
        resp = set_io(SetIORequest.FUN_SET_DIGITAL_OUT, 3, state)
        rospy.sleep(1)
