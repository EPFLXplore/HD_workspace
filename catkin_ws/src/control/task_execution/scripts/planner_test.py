#!/usr/bin/env python

import sys
import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

pose_goal_pub = rospy.Publisher("/arm_control/pose_goal", geometry_msgs.msg.Pose, queue_size=5)


def publish_pose_goal():
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    pose_goal_pub.publish(pose_goal)


def main():
    rospy.init_node("trajectory_planner_node", anonymous=True)

    publish_pose_goal()
    rospy.spin()


if __name__ == "__main__":
    main()
