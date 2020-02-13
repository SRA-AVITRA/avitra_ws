#!/usr/bin/env python

###################################################################################################################
# CODE TO COMPUTE ODOMETRY DATA FROM RAW ENCODER TICKS
# THE MODEL APPLIES TO DIFFERENTIAL DRIVE ROBOTS
# ODOMETRY INFORMATION IS PUBLISHED ON THE "odom" TOPIC AND ROBOT BASE MOVES WITH THE "odom" FRAME AS THE ORIGIN
# TF TREE IS BROADCAST AS FOLLOWS (laser_frame --> base_link--> odom)
###################################################################################################################

import rospy
from math import *
import tf
from nav_msgs.msg import Odometry
from auto_nav.msg import base_params_msg
from perception.msg import array,array_float


#All in MKS
wheel_dia  = 0.154                                                                   # wheel diameter in m
ppr = 540   #135                                                                   # encoders ticks per rotation
bot_dia = 0.475
temp_distance = 0.00
angle = 0
obj_x = 0
obj_y = 0
rpm_L = 0
rpm_R = 0
x = 0
y = 0
theta = 0
flag = False
##################################################################################################################

class Queue:                                                                # queue for storing published raw ticks of the encoders
    def __init__(self):
        self.items = []

    def is_empty(self):
        return self.items == []

    def enqueue(self, data):
        self.items.append(data)

    def dequeue(self):
        return self.items.pop(0)

####################################################################################################################

def base_params_callback(base_params):
    global duty_cycle_L, duty_cycle_R, rpm_R, rpm_L
    queue_left.enqueue(base_params.ticks_L)
    queue_right.enqueue(base_params.ticks_R)
    rpm_L = base_params.desr_rpm_L
    rpm_R = base_params.desr_rpm_R


def camera_coordinates(cam_array):
    global flag, x, y, obj_x, obj_y, theta
    # cam_z = cam_array.array[2]
    # cam_x = cam_array.array[0]
    # count = 1
    # if flag == False:
    #     obj_x = x + cam_z*cos(theta) 
    #     obj_y = y + cam_x*sin(theta) 
    #     flag = True
    # else:
    #     obj_x = obj_x*count + (x + cam_z )
    #     obj_x /= (count + 1)
    #     obj_y = obj_y*count + (y + cam_x )
    #     obj_y /= (count + 1)
    #     count+=1

    obj_x = cam_array.array[2]
    obj_y = cam_array.array[0]
    obj_broadcaster.sendTransform((obj_x, obj_y, 0), (0, 0, 0, 1), rospy.Time.now(), "object_frame", "base_link")

    # flag = True
    

#####################################################################################################################
if __name__ == '__main__':
    global obj_x, obj_y, rpm_R, rpm_L, x, y, theta
    angle = 0
    queue_right = Queue()                                                   # queue objects for right and left encoders
    queue_left = Queue()
    rospy.init_node('avitra_odometry', anonymous=False)
    rospy.Subscriber("base_params", base_params_msg, base_params_callback)                # Subscribing to raw encoder tics and velocities
    rospy.Subscriber("position" ,array_float ,camera_coordinates)
    odom_broadcaster = tf.TransformBroadcaster()
    laser_broadcaster = tf.TransformBroadcaster()
    obj_broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        try:
            if not(queue_left.is_empty()):
                ticks_R = queue_right.dequeue()
                ticks_L = queue_left.dequeue()
                theta =(ticks_R-ticks_L)*pi*wheel_dia/(bot_dia*ppr)                    # calculation of angular displacement from latest orientation
                angle = angle + theta     
                if rpm_L == rpm_R:
                    temp_distance = 0
                else:                                                                         # absolute angular displacement  
                    temp_distance = ((ticks_L+ticks_R)/2)*pi*wheel_dia/ppr           # local linear displacemnt (from previous position)
                x = x + temp_distance*cos(angle)                            # absolute postions w.r.t origin
                y = y + temp_distance*sin(angle)
                print "Angle = " + str((angle*180/3.142)%360) + "\tX = " + str(x) + "\tY = " + str(y) + "\tobj_x = " + str(obj_x) + "\tobj_y = " + str(obj_y)   
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, angle)
            odom_broadcaster.sendTransform(                                                                                 # transformation of robot base_link as computed from odometry data
            (x, y, 0.),
            odom_quat,
            rospy.Time.now(),
            "base_link",
            "odom"
            )
            laser_broadcaster.sendTransform((0, 0, 0.25), (0, 0, 0, 1), rospy.Time.now(), "camera_depth_frame", "base_link")       # fixed transform between robot base and laser scanner
        except Exception as E:
            print "EXCEPTION", E
            continue

