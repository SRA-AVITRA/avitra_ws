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

#OFF-LOAD BEST VALUES *******
# Kp = 2
# Kd = 0
# Ki = 0.4
# pwm_frequency = 200

#on load wheel
# Kp = 1.5
# Kd = 0.05
# Ki = 0.75
# pwm_frequency = 200

#on load 4 wheel
Kp = 2
Kd = 0
Ki = 0.4

curr_rpm_R = []
curr_rpm_L = []

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
    global curr_rpm_R, curr_rpm_L
    if i not in range(0,2000):
        end()
    if desr_rpm != 0:
        ax1.axhline(y=desr_rpm, color='b', linestyle='-')
        ax2.axhline(y=desr_rpm, color='b', linestyle='-')
        ax1.plot(curr_rpm_R, color = 'r')
        ax2.plot(curr_rpm_L, color = 'r')
    print "Kp =", Kp, "\tKd =", Kd, "\tKi =", Ki, "i = ", i
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
    global curr_rpm_R, curr_rpm_L
    desr_rpm = pid_response.desr_rpm
    if pid_response.name = 'MOTOR_L'
        curr_rpm_L.append(pid_response.curr_rpm)
    elif pid_response.name = 'MOTOR_R'
        curr_rpm_R.append(pid_response.curr_rpm)
    i += 1

def end():
    global rpm_R, desr_rpm, Kp, Kd, Ki, alpha, i
    title = "/home/swapnil/avitra_ws/src/auto_nav/observations_for_analysis/plots/rpm_tuning/on_floor_in_bcr/duty_is_pid_4_wheel/forward_desr_rpm"+str(desr_rpm)+"Kp"+str(Kp)+"_Kd"+str(Kd)+"_Ki"+str(Ki)+".png"
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
    
    ax1 = fig.add_subplot(1,2,1)
    ax2 = fig.add_subplot(1,2,2)
    ax1.set_title('MOTOR_L')
    ax2.set_title('MOTOR_R')

    ani = animation.FuncAnimation(fig, animate, interval = 1)
    fig.show()        