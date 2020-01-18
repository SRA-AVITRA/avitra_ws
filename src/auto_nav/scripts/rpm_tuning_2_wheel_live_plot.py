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
Kp = 1.5
Kd = 0.05
Ki = 0.5

i = 0
desr_rpm = 50
curr_rpm_R = []
curr_rpm_L = []
err = []
flag = False

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
    global Kp, Kd, Ki, desr_rpm, curr_rpm_R, curr_rpm_L, i, err, flag
    if i not in range(0,2000):
        end()
    if  flag == True:
        ax1.axhline(y=desr_rpm, color='b', linestyle='-')
        ax2.axhline(y=desr_rpm, color='b', linestyle='-')
        ax1.plot(curr_rpm_R, color = 'r')
        ax2.plot(curr_rpm_L, color = 'r')
        ax3.plot(err, color = 'r')
    print "Kp =", Kp, "\tKd =", Kd, "\tKi =", Ki, "i = ", i
    key = getKey()
    if key == "q":
        end()
    publish_tuna()

def publish_tuna():
    global Kp, Kp, Ki, tuna
    tuna.Kp = Kp
    tuna.Kd = Kd
    tuna.Ki = Ki
    pub.publish(tuna)

def pid_callback(pid_response):
    global desr_rpm, curr_rpm_R, curr_rpm_L, i, flag
    curr_rpm_R.append(pid_response.motor_R)
    curr_rpm_L.append(pid_response.motor_L)
    if pid_response.motor_L != 0:
        flag = True
    else:
        flag = False
    err.append(pid_response.motor_R - pid_response.motor_L)
    i += 1

def end():
    global Kp, Kd, Ki, curr_rpm_R, curr_rpm_L, i
    title = "/home/swapnil/avitra_ws/src/auto_nav/observations_for_analysis/plots/rpm_tuning/on_floor_in_bcr/duty_is_pid_4_wheel_dd_plot/desr_rpm50_Kp"+str(Kp)+"_Kd"+str(Kd)+"_Ki"+str(Ki)+".png"
    Kp, Kd, Ki = 0, 0, 0
    publish_tuna()
    plt.savefig(title)
    plt.show()
    print "DONE"
    exit()

if __name__=="__main__":
    rospy.init_node('rpm_tuning',anonymous=False)
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('tuning', tuning_msg, queue_size=10)          # publisher for teleop_key
    # rospy.Subscriber("pid_response", pid_response , pid_callback)
    rospy.Subscriber("pid_response", velocity_msg, pid_callback)
    tuna = tuning_msg()    
    fig = plt.figure()
    
    ax1 = fig.add_subplot(1,3,1)
    ax2 = fig.add_subplot(1,3,2)
    ax3 = fig.add_subplot(1,3,3)
    ax1.set_title('MOTOR_L rpm')
    ax2.set_title('MOTOR_R rpm')
    ax3.set_title('err')

    ani = animation.FuncAnimation(fig, animate, interval = 1)
    fig.show()        