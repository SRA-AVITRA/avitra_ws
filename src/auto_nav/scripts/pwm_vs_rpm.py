#!/usr/bin/env python

###################################################################################################################
# CODE TO TUNE PID
#   Mode   Plus   Minus
#   Kp      p       l
#   Kd      d       x
#   Ki      i       j
# PRESS 'q' TO EXIT
###################################################################################################################

import struct
import rospy
import sys, select, termios, tty
from auto_nav.msg import velocity_msg
from matplotlib import pyplot as plt

pwm = velocity_msg()
rpm = velocity_msg()

rpmL = []
rpmR = []

fig, axs = plt.subplots(2)

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
    pwm.motor_L = motor_L
    pwm.motor_R = motor_R
    pub_pwm.publish(pwm)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)                                         # read key pressed on keyboard
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_rpm(rpm):
    global rpmL,rpmR
    rpmL.append(rpm.motor_L)
    rpmR.append(rpm.motor_R)

def plot():
    global rpmL, rpmR, axs
    axs[0].plot(pwm.motor_L, rpmL)
    axs[1].plot(pwm.motor_R, rpmR)
    axs[0].set_title('MOTOR_L')
    axs[1].set_title('MOTOR_R')
    rpmL.clear()
    rpmR.clear()
    
def save_plot():    
    plt.show()
    title = "~/avitra_ws/src/auto_nav/plots/pwm_vs_rpm.png"
    fig.savefig(title)

if __name__=="__main__":
    rospy.init_node('teleop',anonymous=False)
    settings = termios.tcgetattr(sys.stdin)
    pub_pwm = rospy.Publisher('teleop', velocity_msg, queue_size=10)
    rospy.Subscriber("rpm", velocity_msg , print_rpm)
    deploy_payloads("r", 0)
    direction = "r"
    speed = 0
    motor_L = 0
    motor_R = 0
    while not rospy.is_shutdown():
        print "direction =", direction, "\tspeed =", speed
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
            plot()
            speed += 1
            if speed > 30:
                speed = 30
        elif key == "h":
            speed -= 1
            if speed < 0:
                speed = 0
        elif key == "x":
            save_plot()
            deploy_payloads("r", 0)
            exit()
        deploy_payloads(direction, speed)
