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
from auto_nav.msg import velocity_msg
from auto_nav.msg import tuning_msg
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

#OFF-LOAD BEST VALUES *******
# Kp = 2
# Kd = 0
# Ki = 0.4

#on load wheel
# Kp = 1.5
# Kd = 0
# Ki = 0.75

i = 0
run = False
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
    
    tuning.Kp = 2
    tuning.Kd = 0
    tuning.Ki = 0.4
    tuning.alpha = 0
    tuning.iTerm_limit = 0
    desr_rpm.motor_L = 10
    desr_rpm.motor_R = -10
    pub_tuning.publish(tuning)
    pub_desr_rpm.publish(desr_rpm)

    # if i not in range(0,2000):
    #     end()
    # ax1.set_xlim(left = i-250, right = i+250)
    # ax2.set_xlim(left = i-250, right = i+250)
    # ax3.set_xlim(left = i-250, right = i+250)
    # ax4.set_xlim(left = i-250, right = i+250)
    # ax1.set_ylim(bottom = 0, top = 100)
    # ax2.set_ylim(bottom = 0, top = 100)
    # ax3.set_ylim(bottom = 0, top = 60)
    # ax4.set_ylim(bottom = 0, top = 60)

    ax1.plot(desr_rpm_R, color = 'b')
    ax2.plot(desr_rpm_L, color = 'b')
    ax1.plot(curr_rpm_R, color = 'r')
    ax2.plot(curr_rpm_L, color = 'r')
    ax3.plot(duty_cycle_R, color = 'r')
    ax4.plot(duty_cycle_L, color = 'r')

    key = getKey()
    if key == "q":
        end()

def base_params_callback(base_params):
    global desr_rpm_L, desr_rpm_R, curr_rpm_R, curr_rpm_L, duty_cycle_L, duty_cycle_R, i
    desr_rpm_L.append(base_params.desr_rpm_L)
    desr_rpm_R.append(base_params.desr_rpm_R)
    curr_rpm_L.append(base_params.curr_rpm_L)
    curr_rpm_R.append(base_params.curr_rpm_R)
    duty_cycle_L.append(base_params.duty_cycle_L)
    duty_cycle_R.append(base_params.duty_cycle_R)
    i += 1

def end():
    global desr_rpm, tuning
    title = "/home/swapnil/avitra_ws/src/auto_nav/observations_for_analysis/plots/rpm_tuning/on_floor_in_bcr/duty_is_pid_term/24_Jan_2020/desr_rpm"+str(desr_rpm.motor_L)+"&"+str(desr_rpm.motor_R)+"_Kp"+str(tuning.Kp)+"_Kd"+str(tuning.Kd)+"_Ki"+str(tuning.Ki)+"_alpha"+str(tuning.alpha)+".png"
    tuning.Kp = 0
    tuning.Kd = 0
    tuning.Ki = 0
    tuning.alpha = 0
    tuning.iTerm_limit = 0
    desr_rpm.motor_L = 0
    desr_rpm.motor_R = 0
    pub_tuning.publish(tuning)
    pub_desr_rpm.publish(desr_rpm)
    print "DONE!"
    # plt.savefig(title)
    plt.show()
    exit()

if __name__=="__main__":
    rospy.init_node('on_load_live_plot',anonymous=False)
    settings = termios.tcgetattr(sys.stdin)
    pub_desr_rpm = rospy.Publisher('desr_rpm', velocity_msg, queue_size=10)
    pub_tuning = rospy.Publisher('tuning', tuning_msg, queue_size=10)
    rospy.Subscriber("base_params", base_params_msg, base_params_callback)
    desr_rpm = velocity_msg()
    tuning = tuning_msg()

    fig = plt.figure()    
    ax1 = fig.add_subplot(2,2,1)
    ax2 = fig.add_subplot(2,2,2)
    ax3 = fig.add_subplot(2,2,3)
    ax4 = fig.add_subplot(2,2,4)
    ax1.set_title('rpm_L')
    ax2.set_title('rpm_R')
    ax3.set_title('duty_cycle_L')
    ax4.set_title('duty_cycle_R')
    ani = animation.FuncAnimation(fig, animate, interval = 1)
    fig.show()        