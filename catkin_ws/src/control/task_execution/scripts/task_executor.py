#!/usr/bin/env python

import rospy
from task_execution.task_classes import *
from task_execution.msg import Task


class Executor:
    def __init__(self):
        self.task = None

    def taskAssignementCallback(self, msg):
        """listens to /arm_control/task_assignment topic"""
        if 0 and self.task is not None:
            return
        if msg.description == "btn":
            self.task = PressButton(msg.pose)
            self.initiateTask()
            self.task = None

    def assignTask(self, task):
        """assigns the task"""
    def initiateTask(self):
        """starts assigned task"""
        self.task.execute()
    def abortTask(self):
        """stops the assigned task"""
    
    def run(self):
        rospy.Subscriber("/arm_control/task_assignment", Task, self.taskAssignementCallback)
        rospy.spin()


if __name__ == "__main__":
    """init ros and subscribe to task commands from manager"""
    try:
        rospy.init_node('HD_control_task_executor', anonymous=True)
        exe = Executor()
        exe.run()
    except rospy.ROSInterruptException:
        rospy.logwarns("task executor crashed")
