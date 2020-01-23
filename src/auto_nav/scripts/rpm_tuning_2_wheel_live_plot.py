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
from auto_nav.msg import base_params_msg
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

#OFF-LOAD BEST VALUES *******
Kp = 2
Kd = 0
Ki = 0.4

#on load wheel
# Kp = 1.5
# Kd = 0
# Ki = 0.75

i = 0
desr_rpm_L = []
desr_rpm_R = []
curr_rpm_R = []
curr_rpm_L = []
duty_cycle_L = []
duty_cycle_R = []

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)                                         # read key pressed on keyboard
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def animate(j):
    global desr_rpm_L, desr_rpm_R, curr_rpm_R, curr_rpm_L, duty_cycle_L, duty_cycle_R, i
    # if i not in range(0,2000):
    #     end()
    ax1.set_xlim(left = i-100, right = i+100)
    ax2.set_xlim(left = i-100, right = i+100)
    ax3.set_xlim(left = i-100, right = i+100)
    ax3.set_xlim(left = i-100, right = i+100)

    ax1.set_ylim(bottom = 0, top = 50)
    ax2.set_ylim(bottom = 0, top = 50)
    ax3.set_ylim(bottom = 0, top = 100)
    ax4.set_ylim(bottom = 0, top = 100)

    ax1.axhline(y=desr_rpm_L, color='b', linestyle='-')
    ax2.axhline(y=desr_rpm_R, color='b', linestyle='-')

    ax1.plot(curr_rpm_R, color = 'r')
    ax2.plot(curr_rpm_L, color = 'r')
    ax3.plot(duty_cycle__R, color = 'r')
    ax4.plot(duty_cycle__L, color = 'r')
    
    key = getKey()
    if key == "q":
        end()
    publish_tuna()

def response_callback(response):
    global desr_rpm_L, desr_rpm_R, curr_rpm_R, curr_rpm_L, duty_cycle_L, duty_cycle_R, i
    desr_rpm_L.append(response.desr_rpm_L)
    desr_rpm_R.append(response.desr_rpm_R)
    curr_rpm_L.append(response.curr_rpm_L)
    curr_rpm_R.append(response.curr_rpm_R)
    duty_cycle_L.append(response.duty_cycle_L)
    duty_cycle_R.append(response.duty_cycle_R)
    i += 1

def end():
    # title = "/home/swapnil/avitra_ws/src/auto_nav/observations_for_analysis/plots/rpm_tuning/on_floor_in_bcr/duty_is_pid_term/20_Jan_2020/4wheel_backward_desr_rpm"+str(desr_rpm)+".png"
    # plt.savefig(title)
    plt.show()
    exit()

if __name__=="__main__":
    rospy.init_node('on_load_live_plot',anonymous=False)
    settings = termios.tcgetattr(sys.stdin)
    rospy.Subscriber("params", base_params_msg, response_callback)
    fig = plt.figure()    
    ax1 = fig.add_subplot(2,2,1)
    ax2 = fig.add_subplot(2,2,2)
    ax3 = fig.add_subplot(2,2,3)
    ax3 = fig.add_subplot(2,2,4)
    ax1.set_title('rpm_L')
    ax2.set_title('rpm_R')
    ax3.set_title('duty_cycle_L')
    ax4.set_title('duty_cycle_R')
    ani = animation.FuncAnimation(fig, animate, interval = 1)
    fig.show()        