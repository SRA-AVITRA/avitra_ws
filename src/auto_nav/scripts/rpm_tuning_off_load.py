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
from auto_nav.msg import pid_response
from matplotlib import pyplot as plt

Kp = 2.5
Kd = 0
Ki = 0.4
iTerm_max = 50
iTerm_min = -50
rpm_R = []
duty_cycle_R = []
desr_rpm = 0

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

def pid_callback(pid_response):
    global rpm_R, desr_rpm
    rpm_R.append(pid_response.curr_rpm)
    # duty_cycle_R.append(pid.duty_cycle)
    desr_rpm = pid_response.desr_rpm

def plot():
    global rpm_R, desr_rpm
    fig, axs = plt.subplots(2)
    plt.plot(rpm_R, color='r')
    plt.axhline(y=desr_rpm, color='b', linestyle='-')
    # axs[0].set_title('rpm_R')
    # axs[1].set_title('duty_cycle_R')
    title = "/home/swapnil/avitra_ws/src/auto_nav/observations_for_analysis/plots/rpm_tuning/off_load_right_motor/duty_is_pid_term/"+str(Kp)+"_"+str(Kd)+"_"+str(Ki)+"_"+str(iTerm_max)+"_"+str(iTerm_min)+".png"
    plt.savefig(title)
    plt.show()

if __name__=="__main__":
    rospy.init_node('rpm_tuning',anonymous=False)
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('tuning', tuning_msg, queue_size=10)          # publisher for teleop_key
    rospy.Subscriber("pid_response", pid_response , pid_callback)
    tuna = tuning_msg()
    while not rospy.is_shutdown():
        print "Kp =", Kp, "\t\tKd =", Kd, "\tKi =", Ki
        key = getKey()
        if key == "p":
            Kp+=0.01
        elif key == "l":
            Kp-=0.01
        elif key == "d":
            Kd+=0.0001
        elif key == "x":
            Kd-=0.0001
        elif key == "i":
            Ki+=0.0005
        elif key == "j":
            Ki-=0.0005
        elif key == "q":
            plot()
            exit()
        publish_tuna()
