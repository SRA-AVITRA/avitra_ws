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
import matplotlib.animation as animation
from matplotlib import style

# Kp = 2.5
# Kd = 2000
# Ki = 0.05

# Kp = 3.5
# Kd = 125
# Ki = 0.25

#BEST VALUES *******
# Kp = 2
# Kd = 0
# Ki = 0.4
# pwm_frequency = 200

#Cum_err/count
Kp = 2
Kd = 0
Ki = 0.4
pwm_frequency = 200

curr_rpm_R = []
duty_cycle_R = []
err_R = []
pTerm_R = []
dTerm_R = []
iTerm_R = []
prev_err_R = []
err_diff = []

i = 0
iTerm_limit = 0
desr_rpm = 0
alpha = 0

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
    global rpm_R, desr_rpm, Kp, Kd, Ki, alpha, i
    if i not in range(0,5000):
        end()
    if desr_rpm != 0:
        ax1.axhline(y=desr_rpm, color='b', linestyle='-')
        ax1.plot(curr_rpm_R, color = 'r')
        ax2.plot(duty_cycle_R, color = 'r')
        ax3.plot(err_R, color = 'r')
        ax4.plot(pTerm_R, color = 'r')
        ax5.plot(dTerm_R, color = 'r')
        ax6.plot(iTerm_R, color = 'r')
        # ax6.plot(err_diff, color = 'r')
    print "Kp =", Kp, "\t\tKd =", Kd, "\tKi =", Ki, "i = ", i
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
        end()
    publish_tuna()

def publish_tuna():
    global Kp, Kp, Ki, tuna
    tuna.Kp = Kp
    tuna.Kd = Kd
    tuna.Ki = Ki
    pub.publish(tuna)

def pid_callback(pid_response):
    global rpm_R, desr_rpm, duty_cycle_R, i, alpha
    
    curr_rpm_R.append(pid_response.curr_rpm)
    duty_cycle_R.append(pid_response.duty_cycle)
    err_R.append(pid_response.err)
    pTerm_R.append(pid_response.pTerm)
    dTerm_R.append(pid_response.dTerm)
    iTerm_R.append(pid_response.iTerm)
    iTerm_limit = pid_response.iTerm_limit
    alpha = pid_response.alpha
    # prev_err_R.append(pid_response.prev_err)
    # err_diff.append(pid_response.err - pid_response.prev_err)
    desr_rpm = pid_response.desr_rpm
    i += 1

def end():
    global rpm_R, desr_rpm, Kp, Kd, Ki, alpha, i
    # title = "/home/swapnil/avitra_ws/src/auto_nav/observations_for_analysis/plots/rpm_tuning/off_load_right_motor/duty_is_pid_term_live/15_Jan_2020/Kp"+str(Kp)+"_Kd"+str(Kd)+"_Ki"+str(Ki)+"_pwm_freq"+str(pwm_frequency)+"_alpha"+str(alpha)+"_iTerm_limit"+str(iTerm_limit)+".png"
    # title = "/home/swapnil/avitra_ws/src/auto_nav/observations_for_analysis/plots/rpm_tuning/off_load_right_motor/duty_is_pid_term/temp.png"
    title = "/home/swapnil/avitra_ws/src/auto_nav/observations_for_analysis/plots/rpm_tuning/off_load_right_motor/duty_is_pid_term_live/16_Jan_2020/cum_err_per_count_samples_5000_Kp"+str(Kp)+"_Kd"+str(Kd)+"_Ki"+str(Ki)+".png"
    Kp, Kd, Ki = 0, 0, 0
    publish_tuna()
    # plt.savefig(title)
    plt.show()
    print "DONE"
    exit()

if __name__=="__main__":
    rospy.init_node('rpm_tuning',anonymous=False)
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('tuning', tuning_msg, queue_size=10)          # publisher for teleop_key
    rospy.Subscriber("pid_response", pid_response , pid_callback)
    tuna = tuning_msg()    
    fig = plt.figure()
    
    # ax1 = fig.add_subplot(1,2,1)
    # ax2 = fig.add_subplot(1,2,2)
    # ax1.set_title('curr_rpm_R')
    # ax2.set_title('duty_cycle_R')

    ax1 = fig.add_subplot(2,3,1)
    ax2 = fig.add_subplot(2,3,2)
    ax3 = fig.add_subplot(2,3,3)
    ax4 = fig.add_subplot(2,3,4)
    ax5 = fig.add_subplot(2,3,5)
    ax6 = fig.add_subplot(2,3,6)

    ax1.set_title('curr_rpm_R')
    ax2.set_title('duty_cycle_R')    
    ax3.set_title('err_R')    
    ax4.set_title('pTerm_R')    
    ax5.set_title('dTerm_R')    
    ax6.set_title('iTerm')    
    
    ani = animation.FuncAnimation(fig, animate, interval = 1)
    fig.show()        