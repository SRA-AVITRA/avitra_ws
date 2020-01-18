#!/usr/bin/env python

###################################################################################################################
# CODE TO CONVERT TWIST TO DESR_RPM
###################################################################################################################

import struct
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from auto_nav.msg import velocity_msg

bot_radius = 0.475/2
wheel_radius = 0.154/2
pi = 3.14159265358
rpm = velocity_msg()

def callback(msg):
    global rpm
    vel_L = (msg.linear.x - bot_radius * msg.angular.z)
    vel_R = (msg.linear.x + bot_radius * msg.angular.z)
    rpm.motor_L = vel_L * 60.0 / (2.0 * pi * wheel_radius);
    rpm.motor_R = vel_R * 60.0 / (2.0 * pi * wheel_radius);
    pub.publish(rpm)

if __name__=="__main__":
    rospy.init_node('autonav',anonymous=False)
    rospy.Subscriber("/cmd_vel", Twist , callback)                        # subscribing to command velocity topic published by move_base
    pub = rospy.Publisher('rpm', velocity_msg, queue_size=10)          # publisher for desr_rpm data
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except Exception as E:
            print "EXCEPTION", E
            continue