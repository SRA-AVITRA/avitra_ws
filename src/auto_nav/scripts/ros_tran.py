#!/usr/bin/env python

###################################################################################################################
# CODE TO DRIVE THE ROBOT VIA TELEOPERATION
# PRESS 'w' TO MOVE FORWARD
# PRESS 's' TO MOVE BACKWARD
# PRESS 'a' TO TURN LEFT ON SPOT
# PRESS 'd' TO TURN RIGHT ON SPOT
# PRESS 'y' TO INCREMENT SPEED BY 1
# PRESS 'h' TO DECREMENT SPEED BY 1
# PRESS 'x' TO EXIT
###################################################################################################################

import struct
import rospy
from std_msgs.msg import String
import sys, select, termios, tty
from auto_nav.msg import velocity_msg

def deploy_payloads(direction, speed):
    global motor_L, motor_R, velocity

    if direction == "w":
        motor_L = speed
        motor_R = -speed
    elif direction == "s":
        motor_L = -speed
        motor_R = speed
    elif direction == "a":
        motor_L = -speed
        motor_R = -speed
    elif direction == "d":
        motor_L = speed
        motor_R = speed
    else:
        motor_L = 0
        motor_R = 0
    velocity.motor_L = motor_L
    velocity.motor_R = motor_R
    pub_velocity.publish(velocity)
    
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
    rospy.init_node('ros_tran',anonymous=False)
    settings = termios.tcgetattr(sys.stdin)
    pub_velocity = rospy.Publisher('teleop', velocity_msg, queue_size=10)          # publisher for teleop_key
    velocity = velocity_msg()
    direction = "r"
    speed = 0
    motor_L = 0
    motor_R = 0
    velocity.command = 0
    deploy_payloads("r", 0)
    while not rospy.is_shutdown():
        print "direction =", direction, "\tspeed =", speed, "\t command =", command
        key = getKey()
        if key == "r":
            direction = "r"
        elif key == "w":
            direction = "w"
        elif key == "s":
            direction = "s"
        elif key == "a":
            direction = "a"
        elif key == "d":
            direction = "d"
        elif key == "y":
            speed += 1
            if speed > 30:
                speed = 30
        elif key == "h":
            speed -= 1
            if speed < 0:
                speed = 0
        elif key == "q"
            velocity.command = 0
        elif key == "z"
            velocity.command = 1
        elif key == "x":
            deploy_payloads("r", 0)
            exit()
        deploy_payloads(direction, speed)
