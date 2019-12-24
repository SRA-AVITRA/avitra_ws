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

rpm_L = []
rpm_R = []

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

def rpm_callback(rpm):
    rpm_L.append(rpm.motor_L)
    rpm_R.append(rpm.motor_R)

def plot():
    for i in range(len(rpm_L)):
        axs[0].scatter(pwm.motor_L, rpm_L[i])
    for i in range(len(rpm_R)):
        axs[1].scatter(pwm.motor_R, rpm_R[i])
    del rpm_L[:]
    del rpm_R[:]

def save_plot():    
    plt.show()
    title = "/home/swapnil/avitra_ws/src/auto_nav/plots/pwm_vs_rpm.png"
    fig.savefig(title)

if __name__=="__main__":
    rospy.init_node('teleop',anonymous=False)
    settings = termios.tcgetattr(sys.stdin)
    pub_pwm = rospy.Publisher('velocity', velocity_msg, queue_size=10)
    rospy.Subscriber("rpm", velocity_msg , rpm_callback)
    pwm = velocity_msg()
    deploy_payloads("r", 0)
    direction = "r"
    speed = 0
    motor_L = 0
    motor_R = 0
    fig, axs = plt.subplots(2)
    axs[0].set_title('MOTOR_L')
    axs[1].set_title('MOTOR_R')
    axs[0].set(xlabel='pwm_L', ylabel='rpm_L')
    axs[1].set(xlabel='pwm_R', ylabel='rpm_R')
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
            if speed > 100:
                speed = 100
        elif key == "h":
            speed -= 1
            if speed < 0:
                speed = 0
        elif key == "x":
            save_plot()
            deploy_payloads("r", 0)
            exit()
        deploy_payloads(direction, speed)
