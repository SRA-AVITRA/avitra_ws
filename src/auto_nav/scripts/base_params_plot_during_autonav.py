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
import sys
import select
import termios
import tty
from auto_nav.msg import base_params_msg
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

# OFF-LOAD BEST VALUES *******
# Kp = 2
# Kd = 0
# Ki = 0.4

# on load wheel
# Kp = 1.5
# Kd = 0
# Ki = 0.75

i = 0
desr_rpm_L = []
desr_rpm_R = []
curr_rpm_R = []
curr_rpm_L = []
# duty_cycle_L = []
# duty_cycle_R = []


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        # read key pressed on keyboard
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def animate(j):

    global desr_rpm_L, desr_rpm_R, curr_rpm_R, curr_rpm_L, duty_cycle_L, duty_cycle_R, i

    ax1.set_xlim(left=i-100, right=i+100)
    ax2.set_xlim(left=i-100, right=i+100)
    # ax3.set_xlim(left=i-100, right=i+100)
    # ax3.set_xlim(left=i-100, right=i+100)
    ax1.set_ylim(bottom=-70, top=70)
    ax2.set_ylim(bottom=-70, top=70)
    # ax3.set_ylim(bottom=100, top=100)
    # ax4.set_ylim(bottom=100, top=100)

    ax1.plot(desr_rpm_R, color='b')
    ax2.plot(desr_rpm_L, color='b')
    ax1.plot(curr_rpm_R, color='r')
    ax2.plot(curr_rpm_L, color='r')
    # ax3.plot(duty_cycle_R, color='r')
    # ax4.plot(duty_cycle_L, color='r')

    key = getKey()
    if key == "q":
        end()


def base_params_callback(base_params):
    global desr_rpm_L, desr_rpm_R, curr_rpm_R, curr_rpm_L, duty_cycle_L, duty_cycle_R, i
    desr_rpm_L.append(base_params.desr_rpm_L)
    desr_rpm_R.append(base_params.desr_rpm_R)
    curr_rpm_L.append(base_params.curr_rpm_L)
    curr_rpm_R.append(base_params.curr_rpm_R)
    # duty_cycle_L.append(base_params.duty_cycle_L)
    # duty_cycle_R.append(base_params.duty_cycle_R)
    i += 1


def end():
    # title = "/home/swapnil/avitra_ws/src/auto_nav/observations_for_analysis/plots/rpm_tuning/on_floor_in_bcr/duty_is_pid_term/20_Jan_2020/4wheel_backward_desr_rpm"+str(desr_rpm)+".png"
    # plt.savefig(title)
    plt.show()
    exit()


if __name__ == "__main__":
    rospy.init_node('on_load_live_plot', anonymous=False)
    settings = termios.tcgetattr(sys.stdin)
    rospy.Subscriber("base_params", base_params_msg, base_params_callback)

    fig = plt.figure()
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2)
    # ax3 = fig.add_subplot(2, 2, 3)
    # ax4 = fig.add_subplot(2, 2, 4)
    ax1.set_title('rpm_L')
    ax2.set_title('rpm_R')
    # ax3.set_title('duty_cycle_L')
    # ax4.set_title('duty_cycle_R')
    ani = animation.FuncAnimation(fig, animate, interval=1)
    fig.show()
