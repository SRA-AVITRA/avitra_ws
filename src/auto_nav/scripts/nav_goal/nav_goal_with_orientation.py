#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped, Pose
from math import *

obj_x, obj_y = 0,0
bot_x, bot_y = 0,0
goal_x, goal_y = 0,0
radius = 0.53

def get_obj_coordinates(point):
    global obj_x, obj_y
    obj_x = point.point.x
    obj_y = point.point.y

def get_bot_coordinates(pose):
    global bot_x, bot_y
    bot_x = pose.position.x
    bot_y = pose.position.y

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py', anonymous = False)
        obj = rospy.Subscriber("bottle", PointStamped, get_obj_coordinates)
        bot = rospy.Subscriber("odom", Pose, get_bot_coordinates)
        rospy.loginfo("Creating Action Client")
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo("Action client created!")

        rospy.loginfo("obj_x = %f, obj_y = %f",obj_x, obj_y)
        rospy.loginfo("bot_x = %f, bot_y = %f",bot_x, bot_y)
        
        delta = pi - atan2(bot_x, bot_y)
        goal_x = 0
        goal_y = 0
        goal_w = cos(delta/2)    #euler to quaternion with pitch and roll as 0
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation.w = goal_w
        rospy.loginfo("goal_x = %f, goal_y = %f, delta = %f", goal_x, goal_y, delta*180/pi)
        client.send_goal(goal)
        rospy.loginfo("Goal 1 sent")
        
        rospy.loginfo("Waiting...")
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Goal 1 Execution Complete!")
        
        obj_side = sqrt(obj_x**2 + obj_y**2)
        bot_side = sqrt(bot_x**2 + bot_y**2)
        third_side = sqrt((obj_x-bot_x)**2 + (obj_y-bot_y)**2)
        
        alpha = atan2(obj_x, obj_y)
        beta = acos((third_side**2 + obj_side**2 - bot_side**2)/(2*third_side*obj_side))
        goal_x = obj_x - radius*sin(alpha-beta)
        goal_y = obj_y - radius*cos(alpha-beta)
        angle = pi/2 - alpha + beta
        goal_w = cos(angle/2)    #euler to quaternion with pitch and roll as 0
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation.w = goal_w
        rospy.loginfo("goal_x = %f, goal_y = %f, angle = %f", goal_x, goal_y, angle*180/pi)
        client.send_goal(goal)
        rospy.loginfo("Goal 2 sent")
        
        rospy.loginfo("Waiting...")
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Goal 2 Execution Complete!")
        
        # client.send_goal(goal,None,None,feedback_cb)
        # state = client.get_state()
        # rospy.loginfo("Goal state : %s", state)     #Possible states: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
        # if state == 9:
        #     rospy.logerr("No goal is active!")
        #     break

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation Interrupted!")
