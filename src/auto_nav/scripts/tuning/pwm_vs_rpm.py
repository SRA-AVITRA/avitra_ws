#!/usr/bin/env python

import struct
import rospy
import sys, select, termios, tty
from auto_nav.msg import velocity_msg
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import xlwt 
from xlwt import Workbook

rpm_L = []
rpm_R = []
sheet_L = 0
sheet_R = 0

def deploy_payloads(direction, speed):
    global motor_L, motor_R
    if direction == "w":
        motor_L = 0
        motor_R = -speed
    elif direction == "s":
        motor_L = 0
        motor_R = speed
    elif direction == "a":
        motor_L = 0
        motor_R = -speed
    elif direction == "d":
        motor_L = 0
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
    rpm_R.append(rpm.motor_R)

def save_records():
    wb.save('/home/swapnil/avitra_ws/src/auto_nav/observations_for_analysis/sheets/xls/rpm_vs_pwm_forward_pwm_'+str(speed)+'.xls') 
    # fig.savefig('/home/swapnil/avitra_ws/src/auto_nav/observations_for_analysis/plots/pwm_vs_rpm.png')
    plt.show()

def animate(i):
    # for i in range(len(rpm_L)):
    #     global sheet_L, sheet_R
    #     axs[0].scatter(rpm_L[i], pwm.motor_L)
    #     sheet1.write(sheet_L, 0, rpm_L[i])
    #     sheet1.write(sheet_L, 1, pwm.motor_L)
    #     sheet_L += 1
    for i in range(len(rpm_R)):
        axs[1].scatter(rpm_R[i], pwm.motor_R)
        sheet1.write(sheet_R, 3, rpm_R[i])
        sheet1.write(sheet_R, 4, pwm.motor_R)
        sheet_R += 1
    # del rpm_L[:]
    del rpm_R[:]

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
        record()
        speed += 1
        if speed > 100:
            speed = 100
    elif key == "h":
        record()
        speed -= 1
        if speed < 32:
            speed = 32
    elif key == "x":
        record()
        deploy_payloads("r", 0)
        save_records()
        exit()
    deploy_payloads(direction, speed)


if __name__=="__main__":
    rospy.init_node('teleop',anonymous=False)
    settings = termios.tcgetattr(sys.stdin)
    pub_pwm = rospy.Publisher('velocity', velocity_msg, queue_size=10)
    rospy.Subscriber("rpm", velocity_msg , rpm_callback)
    pwm = velocity_msg()
    deploy_payloads("w", 0)
    direction = "w"
    speed = 32
    motor_L = 0
    motor_R = 0
    # fig, axs = plt.subplots(2)
    # # axs[0].set_title('MOTOR_L')
    # axs[1].set_title('MOTOR_R')
    # axs[0].set(xlabel='rpm_L', ylabel='pwm_L')
    # axs[1].set(xlabel='rpm_R', ylabel='pwm_R')
    
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)
    ax1.set_title("rpm_R_vs_pwm_R")

    wb = Workbook()
    sheet1 = wb.add_sheet('rpm_vs_pwm')
    ani = animation.FuncAnimation(fig, animate, interval = 1)
    fig.show()