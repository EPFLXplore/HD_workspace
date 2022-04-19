#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Int8MultiArray, Float32, Int8
import keyboard


MOTOR_COUNT = 4
CONTROL_KEYS = ["q", "w", "e", "r", "t", "z", "u"]

motor_state = [0]*MOTOR_COUNT
vel = 0

angles_pub = rospy.Publisher('HD_angles', Int8MultiArray, queue_size=10)
vel_pub = rospy.Publisher('HD_ManualVelocity', Float32, queue_size=10)


def publish_state():
    msg = Int8MultiArray()
    msg.data = motor_state[:]
    angles_pub.publish(msg)


def publish_vel():
    #print("publishing vel     ", vel)
    msg = Float32()
    msg.data = vel
    vel_pub.publish(msg)


def get_inputs():
    global vel, motor_state
    motor_state = [int(keyboard.is_pressed(CONTROL_KEYS[i])) for i in range(MOTOR_COUNT)]
    vel_step = 0.125
    if keyboard.is_pressed("up"):
        vel = min(vel+vel_step, 1)
    if keyboard.is_pressed("down"):
        vel = max(vel-vel_step, -1)


def talker():
    rospy.init_node('CS_sim_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.logwarn("CS_sim started")
    while not rospy.is_shutdown():
        get_inputs()
        publish_state()
        publish_vel()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass