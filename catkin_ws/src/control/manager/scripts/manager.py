#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Float32MultiArray, Int8MultiArray, Float32, Int8


class Manager:
    AUTONOMOUS = 0
    SEMIAUTONOMOUS = 1
    MANUAL_INVERSE = 2
    MANUAL_DIRECT = 3

    def __init__(self):
        self.velocity = 0
        self.received_velocity_at = time.time()
        self.velocity_expiration = .5   # seconds
        self.direct_command = []
        self.mode = self.MANUAL_DIRECT
        self.target_mode = self.MANUAL_DIRECT
        self.mode_transitioning = False

    def modeCallback(self, msg):# Int8):
        """listens to HD_mode topic published by CS"""
        self.target_mode = msg.data
        self.mode_transitioning = True

    def taskCmdCallback(self, msg):# Task):
        """listens to task assignement topic published by detection"""

    def manualCmdCallback(self, msg):# Int8MultiArray):
        """listens to HD_InvManual_Coord topic"""

    def directCmdCallback(self, msg):# Int8MultiArray):
        """listens to HD_Angles topic"""
        self.direct_command = msg.data

    def manualVelocityCallback(self, msg):# Float32):
        """listens to HD_ManualVelocity topic"""
        #rospy.logwarn("received velocity   " + str(msg.data))
        self.velocity = msg.data
        self.received_velocity_at = time.time()

    def send_task_cmd(self):
        """sends the last task command to the task executor and locks any other command until completion"""
    def send_manual_cmd(self):
        """sends the last manual command to the manual control and locks any other command until completion"""

    def send_direct_cmd(self):
        """sends the last direct command to the motor control and locks any other command until completion"""
        if not self.velocity_command_old():
            msg = Float32MultiArray()
            msg.data = self.format_direct_command()
            #rospy.logwarn(str(msg.data))
            self.manual_cmd_pub.publish(msg)

    def updateWorld(self):
        """sends a world update to the trajectory planner"""

    def velocity_command_old(self):
        return time.time()-self.received_velocity_at > self.velocity_expiration

    def format_direct_command(self):
        return [cmd*self.velocity for cmd in self.direct_command]

    def normal_loop_action(self):
        if self.mode == self.AUTONOMOUS:
            pass
        elif self.mode == self.MANUAL_INVERSE or self.mode == self.SEMIAUTONOMOUS:
            pass
        elif self.mode == self.MANUAL_DIRECT:
            self.send_direct_cmd()

    def transition_loop_action(self):
        if self.mode == self.AUTONOMOUS:
            pass
        elif self.mode == self.MANUAL_INVERSE or self.mode == self.SEMIAUTONOMOUS:
            pass
        elif self.mode == self.MANUAL_INVERSE:
            pass

        transition_condition = True
        if transition_condition:
            self.mode_transitioning = False
            self.mode = self.target_mode

    def run(self):
        """main"""
        self.manual_cmd_pub = rospy.Publisher('/arm_control/manual_cmd', Float32MultiArray, queue_size=10)
        rospy.Subscriber("HD_angles", Int8MultiArray, self.directCmdCallback)
        rospy.Subscriber("HD_ManualVelocity", Float32, self.manualVelocityCallback)
        #rospy.init_node('HD_control_manager', anonymous=True)
        rate = rospy.Rate(25)   # 25hz
        rospy.logwarn("manager started")
        while not rospy.is_shutdown():
            if self.mode_transitioning:
                self.transition_loop_action()
            else:
                self.normal_loop_action()
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('HD_control_manager', anonymous=True)
        m = Manager()
        m.run()
        rospy.logwarn("manager finished ????")
    except rospy.ROSInterruptException:
        rospy.logwarns("manager crashed")
        pass
