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

#All in MKS
# wheel diameter in m
wheel_dia = 0.154
ppr = 540  # 135                                                                   # encoders ticks per rotation
bot_dia = 0.475
temp_distance = 0.00
angle = 0
rpm_L = 0
rpm_R = 0

##################################################################################################################


# queue for storing published raw ticks of the encoders
class Queue:
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
    global rpm_L, rpm_R
    queue_left.enqueue(base_params.ticks_L)
    queue_right.enqueue(base_params.ticks_R)
    rpm_L = base_params.desr_rpm_L
    rpm_R = base_params.desr_rpm_R


#####################################################################################################################
if __name__ == '__main__':
    x = 0
    y = 0
    theta = 0
    angle = 0
    # queue objects for right and left encoders
    queue_right = Queue()
    queue_left = Queue()
    rospy.init_node('avitra_odometry', anonymous=False)
    # Subscribing to raw encoder tics and velocities
    rospy.Subscriber("base_params", base_params_msg, base_params_callback)
    odom_broadcaster = tf.TransformBroadcaster()
    laser_broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        try:
            if not(queue_left.is_empty()):
                ticks_R = queue_right.dequeue()
                ticks_L = queue_left.dequeue()
                # calculation of angular displacement from latest orientation
                theta = (ticks_R-ticks_L)*pi*wheel_dia/(bot_dia*ppr)
                # absolute angular displacement
                angle = angle + theta
                if rpm_L == rpm_R and rpm_L != 0:
                    temp_distance = 0
                else:
                    # local linear displacemnt (from previous position)
                    temp_distance = ((ticks_L+ticks_R)/2)*pi*wheel_dia/ppr
                # absolute postions w.r.t origin
                x = x + temp_distance*cos(angle)
                y = y + temp_distance*sin(angle)
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, angle)
            odom_broadcaster.sendTransform(                                                                                 # transformation of robot base_link as computed from odometry data
                (x, y, 0.),
                odom_quat,
                rospy.Time.now(),
                "base_link",
                "odom"
            )
            laser_broadcaster.sendTransform((0, 0, 0.25), (0, 0, 0, 1), rospy.Time.now(
            ), "camera_depth_frame", "base_link")       # fixed transform between robot base and laser scanner
        except Exception as E:
            print "EXCEPTION", E
            continue
