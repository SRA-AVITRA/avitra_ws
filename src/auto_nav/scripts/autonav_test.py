#!/usr/bin/env python

###################################################################################################################
# CODE TO GIVE RPM VIA TWIST
# PRESS 'y' TO INCREMENT LINEAR_X BY 5rpm
# PRESS 'h' TO DECREMENT LINEAR_X BY 5rpm
# PRESS 'i' TO INCREMENT ANGULAR_Z BY 5rpm
# PRESS 'k' TO DECREMENT ANGULAR_Z BY 5rpm
# PRESS 'r' TO RESET THE VALUES
# PRESS 'x' TO EXIT
###################################################################################################################

import struct
import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

bot_radius = 0.475/2
wheel_radius = 0.154/2
pi = 3.14159265358

def publish(linear_x, angular_z):
    global twist
    linear_x *= 2 * pi * wheel_radius / 60 
    angular_z *= 2 * pi * wheel_radius / (bot_radius * 60) 
    twist.linear.x = linear_x
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = angular_z
    pub.publish(twist)
    
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)                                         # read key pressed on keyboard
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    rospy.init_node('autonav_test',anonymous=False)
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)          # publisher for Twist data
    twist = Twist()
    x = 0
    z = 0
    publish(x, z)
    while not rospy.is_shutdown():
        print "x =", x, "\tz =", z
        key = getKey()
        if key == "y":
            x += 5
        elif key == "h":
            x -= 5
        if key == "i":
            z += 5
        elif key == "k":
            z -= 5
        elif key == "r":
            x = 0
            z = 0
        elif key == "x":
            publish(0, 0)
            exit()
        publish(x, z)
