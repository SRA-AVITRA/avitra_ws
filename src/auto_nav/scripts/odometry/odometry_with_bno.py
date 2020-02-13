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
from auto_nav.msg import ticks_msg
from std_msgs.msg import Float32

#All in MKS
wheel_dia = 0.154
ppr = 540  # 135*4
bot_dia = 0.475
temp_distance = 0.00
angle_enc = 0
angle_imu = 0
initial_angle_imu = 0
imu_flag = False
##################################################################################################################


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


def ticks_callback(ticks_array):
    queue_left.enqueue(ticks_array.motor_L)
    queue_right.enqueue(ticks_array.motor_R)


def yaw_callback(yaw):
    global angle_imu #, imu_flag,initial_angle_imu
    # if imu_flag == True:
    #     angle_imu = initial_angle_imu - yaw.data
    # else:
    #    initial_angle_imu = yaw.data
    #     angle = yaw.data
    #     imu_flag = True
    angle_imu = yaw.data


#####################################################################################################################
if __name__ == '__main__':
    angle_enc = 0
    angle_imu = 0
    x_enc, y_enc = 0, 0
    x_imu, y_imu = 0, 0
    theta = 0
    queue_right = Queue()
    queue_left = Queue()
    rospy.init_node('avitra_odometry', anonymous=True)
    rospy.Subscriber("ticks", ticks_msg, ticks_callback)
    rospy.Subscriber("yaw", Float32, yaw_callback)
    odom_broadcaster = tf.TransformBroadcaster()
    laser_broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        try:
            if not(queue_left.is_empty()):
                ticksR = queue_right.dequeue()
                ticksL = queue_left.dequeue()
                theta = (ticksR-ticksL)*pi*wheel_dia/(bot_dia*ppr)
                angle_enc = angle_enc + theta
                temp_distance = ((ticksL+ticksR)/2)*pi*wheel_dia/ppr
                x_enc = x_enc + temp_distance*cos(angle_enc)
                y_enc = y_enc + temp_distance*sin(angle_enc)
                x_imu = x_imu + temp_distance*cos(radians(angle_imu))
                y_imu = y_imu + temp_distance*sin(radians(angle_imu))
                print "ENCODER: angle = " + str(degrees(angle_enc)), "x = ", str(x_enc), "y = ", str(y_enc), "IMU: angle = " + str(angle_imu), "x = ", str(x_imu), "y = ", str(y_imu)

            odom_quat = tf.transformations.quaternion_from_euler(
                0, 0, angle_enc)
            odom_broadcaster.sendTransform(
                (x_enc, y_enc, 0.), odom_quat, rospy.Time.now(), "base_link", "odom")
            laser_broadcaster.sendTransform(
                (0, 0, 0.25), (0, 0, 0, 1), rospy.Time.now(), "camera_depth_frame", "base_link")
        except Exception as E:
            print "EXCEPTION", E
            continue
