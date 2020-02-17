#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped
from math import *

def movebase_client():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    print "action client created\n"

    goal_x = float(raw_input("Enter Goal x = "))
    goal_y = float(raw_input("Enter Goal y = "))
    angle = float(raw_input("Enter Goal angle = "))
    angle *= pi/180          #degree to radian
    goal_w = cos(angle/2)    #euler to quaternion with pitch and roll as 0

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.orientation.w = goal_w
    
    client.send_goal(goal)
    print "goal sent\n"
    wait = client.wait_for_result()
    print "waiting...\n"
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py', anonymous = False)
        result = movebase_client()
        while(True):
            movebase_client()
            print "execution complete\n"

        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
