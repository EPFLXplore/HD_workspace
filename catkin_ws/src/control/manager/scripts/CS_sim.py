#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Int8MultiArray, Float32, Int8


motor_state = 0
vel = 0

angles_pub = rospy.Publisher('HD_angles', Int8MultiArray, queue_size=10)
vel_pub = rospy.Publisher('HD_ManualVelocity', Float32, queue_size=10)


def state_callback(msg):
    global motor_state
    motor_state = msg.data


def vel_callback(msg):
    global vel
    vel = msg.data


def publish_state():
    msg = Int8MultiArray()
    msg.data = [motor_state]
    angles_pub.publish(msg)


def publish_vel():
    #print("publishing vel     ", vel)
    msg = Float32()
    msg.data = vel
    vel_pub.publish(msg)


def talker():
    rospy.init_node('CS_sim_node', anonymous=True)
    rospy.Subscriber("state_cmd", Int8, state_callback)
    rospy.Subscriber("vel_cmd", Float32, vel_callback)
    rate = rospy.Rate(10) # 10hz
    rospy.logwarn("CS_sim started")
    while not rospy.is_shutdown():
        publish_state()
        publish_vel()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass