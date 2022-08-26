#!/usr/bin/env python

import sys
import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from task_execution.srv import PoseGoal, PoseGoalResponse, JointGoal, JointGoalResponse


pose_goal_pub = rospy.Publisher("/arm_control/pose_goal", geometry_msgs.msg.Pose, queue_size=5)
joint_goal_pub = rospy.Publisher("/arm_control/joint_goal", std_msgs.msg.Float64MultiArray, queue_size=5)


def req_joint_goal(angle):
    rospy.wait_for_service('/arm_control/joint_goal')
    try:
        proxy = rospy.ServiceProxy('/arm_control/joint_goal', JointGoal)
        cmd_id = 0
        goal = std_msgs.msg.Float64MultiArray()
        goal.data = [angle, 0, 0, 0, 0, 0]
        resp = proxy(cmd_id, goal)
        print(resp.ok)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def req_pose_goal(x, k):
    rospy.wait_for_service('/arm_control/pose_goal')
    try:
        proxy = rospy.ServiceProxy('/arm_control/pose_goal', PoseGoal)
        cmd_id = 0
        goal = geometry_msgs.msg.Pose()
        """goal.orientation.w = 1.0
        goal.position.x = 0.4
        goal.position.y = 0.1
        goal.position.z = 0.4"""
        if x == "x":
            goal.orientation.x = k
        elif x == "y":
            goal.orientation.y = k
        elif x == "z":
            goal.orientation.z = k
        elif x == "w":
            goal.orientation.w = k
        resp = proxy(cmd_id, goal, False, x)
        print(resp.ok)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def publish_joint_goal(angle):
    msg = std_msgs.msg.Float64MultiArray()
    msg.data = [angle,0,0,0,0,0]
    joint_goal_pub.publish(msg)

def publish_pose_goal():
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    pose_goal_pub.publish(pose_goal)


def main():
    rospy.init_node("trajectory_planner_node", anonymous=True)

    if int(sys.argv[1]) == 1:
        req_joint_goal(int(sys.argv[2]))
    else:
        req_pose_goal(sys.argv[2], int(sys.argv[3]))


if __name__ == "__main__":
    main()
