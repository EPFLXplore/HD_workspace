#!/usr/bin/env python

import time
import rospy
import std_msgs.msg


joint_goal_pub = rospy.Publisher("/arm_control/joint_goal", std_msgs.msg.Float64MultiArray)


def send_joint_goal():
    msg = [1,0,0,0,0,0]
    joint_goal_pub.publish(msg)


def main():
    input()
    send_joint_goal()

    time.sleep(.5)


