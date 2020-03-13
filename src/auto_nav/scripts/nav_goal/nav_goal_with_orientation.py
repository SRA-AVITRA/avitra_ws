#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped, Pose
from math import *
import numpy as np

obj_x, obj_y = 0,0
bot_x, bot_y, bot_w = 0,0,0
goal_x, goal_y = 0,0
radius = 0.4

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return qx, qy, qz, qw

def get_obj_coordinates(point):
    global obj_x, obj_y
    obj_x = point.point.x
    obj_y = point.point.y

def get_bot_coordinates(pose):
    global bot_x, bot_y, bot_w
    bot_x = pose.position.x
    bot_y = pose.position.y
    bot_w = pose.orientation.x

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py', anonymous = False)
        obj = rospy.Subscriber("bottle", PointStamped, get_obj_coordinates)
        bot = rospy.Subscriber("odom", Pose, get_bot_coordinates)
        rospy.loginfo("Creating Action Client")
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo("Action client created!")

        rospy.loginfo("obj_x = %f, obj_y = %f", obj_x, obj_y)
        rospy.loginfo("bot_x = %f, bot_y = %f", bot_x, bot_y)

        angle = atan2(obj_y-bot_y, obj_x-bot_x)
        goal_x = obj_x - radius*cos(angle)
        goal_y = obj_y - radius*sin(angle)
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = euler_to_quaternion(angle, 0, 0)
        rospy.loginfo("goal_x = %f, goal_y = %f, angle = %f", goal_x, goal_y, angle*180/pi)
        client.send_goal(goal)
        rospy.loginfo("Goal 1 sent")
        
        rospy.loginfo("Waiting...")
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Goal 1 Execution Complete!")     
            rospy.loginfo("error in x = %f cm, error in y = %f cm", (goal_x-bot_x)*100, (goal_y-bot_y)*100)
            # rospy.loginfo("error in angle = %f deg", bot_w)

        # client.send_goal(goal,None,None,feedback_cb)
        # state = client.get_state()
        # rospy.loginfo("Goal state : %s", state)     #Possible states: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
        # if state == 9:
        #     rospy.logerr("No goal is active!")
        #     break

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation Interrupted!")
