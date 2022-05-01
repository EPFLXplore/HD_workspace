#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32MultiArray, Int8MultiArray, Float32, Int8
from sensor_msgs.msg import JointState


MOTOR_COUNT = 1
LIMITS = [0, 3]

class Executor:
    def __init__(self):
        self.direction = 1
        self.motor_state = [0]*MOTOR_COUNT
        self.vel = 0

        self.real_pos = [0]*MOTOR_COUNT

        self.angles_pub = rospy.Publisher('HD_angles', Int8MultiArray, queue_size=10)
        self.vel_pub = rospy.Publisher('HD_ManualVelocity', Float32, queue_size=10)
        
        self.limit_reached = False
        self.limit_reached_at = 0
        self.wating_time = .5

    def telem_callback(self, msg):
        self.real_pos = msg.position

    def publish_state(self):
        msg = Int8MultiArray()
        msg.data = self.motor_state[:]
        self.angles_pub.publish(msg)

    def publish_vel(self):
        msg = Float32()
        msg.data = self.vel
        self.vel_pub.publish(msg)

    def waited_enough(self):
        return (time.time()-self.limit_reached_at) > self.waiting_time

    def update_task(self):
        if not(self.limit_reached) and self.direction*self.real_pos[0] > self.direction*LIMITS[(self.direction+1)//2]:
            self.limit_reached = True
            self.limit_reached_at = time.time()
        elif self.limit_reached and self.waited_enough():


    def main(self):
        rospy.init_node('CS_sim_node', anonymous=True)
        rospy.Subscriber("/arm_control/joint_telemetry", JointState, self.telem_callback)
        rate = rospy.Rate(10) # 10hz
        rospy.logwarn("CS_sim started")
        while not rospy.is_shutdown():
            self.publish_state()
            self.publish_vel()
            rate.sleep()

if __name__ == '__main__':
    try:
        Executor().main()
    except rospy.ROSInterruptException:
        pass
