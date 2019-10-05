#!/usr/bin/env python

#################################################################
'''
velocity is published to /teleop and published to /ticks 
'''
#################################################################

import struct
import rospy
from auto_nav.msg import ticks_msg, velocity_msg

velocity = velocity_msg()

def callback(msg):
    print("ticks_l = ", msg.motor_L, "ticks_r = ", msg.motor_R)
    global velocity
    if(velocity.motor_L < 1000):
        velocity.motor_L+=1
    if(velocity.motor_R < 1000):
        velocity.motor_R+=1

if __name__=="__main__":
    rospy.init_node('rosserial',anonymous=False)
    rospy.Subscriber("ticks", ticks_msg, callback)
    pub = rospy.Publisher('teleop', velocity_msg, queue_size=10)
    rate = rospy.Rate(10)   #10Hz
    velocity.motor_L = 0
    velocity.motor_R = 0
    while not rospy.is_shutdown():
        try:
            pub.publish(velocity)
            rate.sleep()
        except KeyboardInterrupt:
            break