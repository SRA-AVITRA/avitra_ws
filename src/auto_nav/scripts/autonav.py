#!/usr/bin/env python

##############################################################################################
# CODE TO MAP ROBOT VELOCITIES TO MOTOR PWM
# THE PWM VALUES ARE SENT VIA ROSSERIAL TO THE BASE MICROCONTROLLER
##############################################################################################

import rospy
from geometry_msgs.msg import Twist
from auto_nav.msg import velocity_msg
pwmL = 0
pwmR = 0
pwmF = 0
pwmB = 0

##############################################################################################

def callback(cmd):
	pwmL = cmd.linear.x													# robot velocities
	pwmR = -cmd.linear.x												# robot velocities
	if pwmL*pwmR < 0:
		pwmF = 0
		pwmB = 0
	else:
		pwmF = -1
		pwmB = -1
	# pwm_values.data = [pwmF, pwmB, pwmL,pwmR]
	pwm_values.motor_F = pwmF;
	pwm_values.motor_B = pwmB;
	pwm_values.motor_L = pwmL;
	pwm_values.motor_R = pwmR;
	pub.publish(pwm_values)

################################################################################################

if __name__ == '__main__':
	rospy.init_node('auto_nav', anonymous=True)
	rospy.Subscriber("/cmd_vel", Twist , callback)						# subscribing to command velocity topic published by move_base
	pub = rospy.Publisher("/motor_pwm", velocity_msg , queue_size = 10)
	pwm_values = velocity_msg()
	rospy.spin()
