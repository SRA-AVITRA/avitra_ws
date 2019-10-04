#!/usr/bin/env python

import struct
import rospy
from std_msgs.msg import String
import sys, select, termios, tty
from auto_nav.msg import duty_msg

def deploy_payloads(direction, speed):
    global motor_L, motor_R
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
    duty.motor_L = motor_L
    duty.motor_R = motor_R
    pub_duty.publish(duty)
    print("*********")
    
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
    deploy_payloads("r", 0)
    direction = "r"
    speed = 0
    motor_L = 0
    motor_R = 0
    pub_duty = rospy.Publisher('teleop', duty_msg, queue_size=10)          # publisher for teleop_key
    duty = duty_msg()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print("direction = ", direction, "\tspeed = ", speed)
        key = getKey()
        if key == "r":
            direction = "r"
            deploy_payloads(direction, speed)
        elif key == "w":
            direction = "w"
            deploy_payloads(direction, speed)
        elif key == "s":
            direction = "s"
            deploy_payloads(direction, speed)
        elif key == "a":
            direction = "a"
            deploy_payloads(direction, speed)
        elif key == "d":
            direction = "d"
            deploy_payloads(direction, speed)
        elif key == "y":
            speed += 1
            if speed > 30:
                speed = 30
            if direction != "r":
                deploy_payloads(direction, speed)
        elif key == "h":
            speed -= 1
            if speed < 0:
                speed = 0
            if direction != "r":
                deploy_payloads(direction, speed)