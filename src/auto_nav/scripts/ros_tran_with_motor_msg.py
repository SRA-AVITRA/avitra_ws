#!/usr/bin/env python

###################################################################################################################
# CODE TO DRIVE THE ROBOT VIA TELEOPERATION
# PRESS 'w' TO MOVE FORWARD
# PRESS 's' TO MOVE BACKWARD
# PRESS 'a' TO TURN LEFT ON SPOT
# PRESS 'd' TO TURN RIGHT ON SPOT
# PRESS 'y' TO INCREMENT RPM BY 1
# PRESS 'h' TO DECREMENT RPM BY 1
# PRESS 'z' TO START AUTONAV
# PRESS 'esc' TO EXIT
###################################################################################################################

import struct
import rospy
from std_msgs.msg import String
import sys, select, termios, tty
from auto_nav.msg import motor_msg
import pygame
import sys

def initialize_pygame():
    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    pygame.key.set_repeat(10,50)

def deploy_payloads(direction, rpm):
    global motor_L, motor_R, rpm_msg
    if direction == "d":
        motor_L = rpm
        motor_R = rpm
    elif direction == "a":
        motor_L = -rpm
        motor_R = -rpm    
    elif direction == "w":
        motor_L = rpm
        motor_R = -rpm
    elif direction == "s":
        motor_L = -rpm
        motor_R = rpm
    else:
        motor_L = 0
        motor_R = 0
    rpm_msg.motor_L = motor_L
    rpm_msg.motor_R = motor_R
    pub_rpm_msg.publish(rpm_msg)
    
if __name__=="__main__":
    rospy.init_node('ros_tran',anonymous=False)
    pub_rpm_msg = rospy.Publisher('teleop', motor_msg, queue_size=10)          # publisher for teleop_key
    rate=rospy.Rate(100)
    initialize_pygame()
    rpm_msg = motor_msg()
    deploy_payloads("r", 0)
    direction = "r"
    rpm = 0
    motor_L = 0
    motor_R = 0
    change_flag = True
    while not rospy.is_shutdown():
        print "\ndirection =", direction, "\trpm =", rpm, "\tcommand =", rpm_msg.command
        pressed = pygame.key.get_pressed()
        w_held = pressed[pygame.K_w] or pressed[pygame.K_w]
        a_held = pressed[pygame.K_a] or pressed[pygame.K_a]
        s_held = pressed[pygame.K_s] or pressed[pygame.K_s]
        d_held = pressed[pygame.K_d] or pressed[pygame.K_d]
        z_held = pressed[pygame.K_z] or pressed[pygame.K_z]
        if w_held:
            direction = "w"
        elif a_held:
            direction = "a"
        elif s_held:
            direction = "s"
        elif d_held:
            direction = "d"
        elif z_held:
            rpm_msg.command = 1
        else:
            direction = "r"
            deploy_payloads("r", 0)
            rpm_msg.command = 0
        for event in pygame.event.get():            
            if event.type == pygame.KEYDOWN:
                if event.type == pygame.QUIT:
                    sys.exit()
                if event.key == pygame.K_ESCAPE:
                    sys.exit()
                if event.key == pygame.K_y and change_flag:
                    rpm += 1
                    if rpm == 1:
                        rpm = 10      # little less than the starting rpm
                    if rpm > 40:
                        rpm = 40
                elif event.key == pygame.K_h and change_flag:
                    rpm -= 1
                    if rpm < 0:
                        rpm = 0
                if z_held:
                    rpm_msg.command = 1
                if change_flag:
                    change_flag = False
            if event.type == pygame.KEYUP:
                change_flag = True
            deploy_payloads(direction, rpm)
        rate.sleep()
