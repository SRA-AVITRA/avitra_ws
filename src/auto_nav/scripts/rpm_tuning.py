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
from auto_nav.msg import tuning_msg
from auto_nav.msg import velocity_msg
from matplotlib import pyplot as plt

Kp = 0.003
Kd = 0.75
Ki = 0.0
rpmL=[]
rpmR=[]

def publish_tuna():
    global Kp, Kp, Ki, tuna
    tuna.Kp = Kp
    tuna.Kd = Kd
    tuna.Ki = Ki
    pub.publish(tuna)

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
    rpmL.append(rpm.motorL)
    rpmR.append(rpm.motorR)

def plot():
    global rpmL, rpmR
    plt.plot(rpmL)
    plt.plot(rpmR)
    plt.show()



if __name__=="__main__":
    rospy.init_node('rpm_tuning',anonymous=False)
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('tuning', tuning_msg, queue_size=10)          # publisher for teleop_key
    rospy.Subscriber("/rpm", velocity_msg , print_rpm)
    tuna = tuning_msg()
    while not rospy.is_shutdown():
        print "Kp =", Kp, "\tKd =", Kd, "\tKi =", Ki
        key = getKey()
        if key == "p":
            Kp+=0.0005
        elif key == "l":
            Kp-=0.0005
        elif key == "d":
            Kd+=0.05
        elif key == "x":
            Kd-=0.05
        elif key == "i":
            Ki+=0.001
        elif key == "j":
            Ki-=0.001
        elif key == "q":
            plot()
            exit()
        publish_tuna()
