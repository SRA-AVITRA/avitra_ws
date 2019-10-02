#!/usr/bin/env python

###################################################################################################################
'''
CODE TO TRANSFER DATA BETWEEN ESP AND JETSON
'''
###################################################################################################################
import serial
import struct
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from auto_nav.msg import uart_ticks_msg, uart_velocity_msg

# 425.16 ticks per (2*pi) radians
# ((pi/6)/(2*pi))*425.16 = 36 ticks for (pi/6)
motion = 0
speed = 0
speed_step = 0.005
dc_motor_L = 0
dc_motor_R = 0

def deploy_payloads(motor_L_dc, motor_R_dc, speed):
	motor_L_dc *= speed
	if motor_L_dc < -40:
		motor_L_dc = -40
	elif motor_L_dc > 40:
		motor_L_dc = 40
	motor_L_payload = 41 + motor_L_dc

	if motor_R_dc < -40:
		motor_R_dc = -40
	elif motor_R_dc > 40:
		motor_R_dc = 40
	motor_R_payload = 169 - motor_R_dc #128 + 41
	ser.write([int(motor_L_payload)])
	ser.write([int(motor_R_payload)])
	# print "*********",
	
def getKey():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)                                         # read key pressed on keyboard
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def sign_of(x):
	if x > 0:
		return 1
	elif x == 0:
		return 0
	else:
		return -1

def cmd_vel_cb(cmd):
	global lin_x, ang_z, dc_motor_L, dc_motor_R, motion
	lin_x = cmd.linear.x													# robot velocities
	ang_z = cmd.angular.z
	pwmL_int = (lin_x + 0.35*ang_z)*79.1
	pwmR_int = (lin_x - 0.35*ang_z)*79.1									# computing individual motor speeds
	dc_motor_L = 85*sign_of(pwmL_int) + 0.3*(pwmL_int)
	dc_motor_R = 85*sign_of(pwmR_int) + 0.3*(pwmR_int)								# pwm mapping depending on architecture and requirement
	# if motion == 1:
	# 	deploy_payloads(dc_motor_L, dc_motor_R, speed)
	# else:
	# 	deploy_payloads(0, 0, 0)

if __name__=="__main__":
	rospy.init_node('read_from_cmd_vel_write_to_uart',anonymous=False)
	try:
		ser = serial.Serial('/dev/ttyUSB0')
	except serial.serialutil.SerialException:
		ser = serial.Serial('/dev/ttyUSB1')
	ser.baudrate = 115200
	settings = termios.tcgetattr(sys.stdin)

	deploy_payloads(0, 0, 0)

	pub_ticks = rospy.Publisher('ticks', uart_ticks_msg, queue_size=10)          # publisher for teleop_key
	pub_velocity = rospy.Publisher("velocity", uart_velocity_msg, queue_size=10)
	rospy.Subscriber("/cmd_vel", Twist , cmd_vel_cb)						# subscribing to command velocity topic published by move_base
	rate = rospy.Rate(10)
	uart_ticks = uart_ticks_msg()
	l_received_flag = 0
	r_received_flag = 0
	while not rospy.is_shutdown():
		while(ser.in_waiting):
			payload_c = ser.read()
			if payload_c != '':
				payload = struct.unpack('>B', payload_c)[0]
				if payload < 128:   #Left
					uart_ticks.motor_L = payload - 41
					if r_received_flag:
						pub_ticks.publish(uart_ticks)
						r_received_flag = 0
					else:
						l_received_flag = 1
				else:   #Right
					uart_ticks.motor_R = 169 - payload
					if l_received_flag:
						pub_ticks.publish(uart_ticks)
						l_received_flag = 0
					else:
						r_received_flag = 1
		key = getKey()
		if key == "w":
			motion = 1
		elif key == "y":
			speed += speed_step
			if speed > 1:
				speed = 1
		elif key == "h":
			speed -= speed_step
			if speed < -1:
				speed = -1
		elif key != "":
			motion = 0
			deploy_payloads(0, 0, 0)
			print "\nKILL\t key = >", key, "< \tspeed = ", speed, "\tdc_motor_L = ", dc_motor_L, "\tdc_motor_R =- ", dc_motor_R
			if key == "x" or key == "X":
				print "\nEND"
				exit()
		print "\n key = >", key, "<\tspeed = ", speed, "\tmotion: ", motion, "\tdc_motor_L = ", dc_motor_L, "\tdc_motor_R =- ", dc_motor_R, "\tticks L: ", uart_ticks.motor_L, "\tR: ", uart_ticks.motor_R	

		if motion == 1:
			deploy_payloads(dc_motor_L, dc_motor_R, speed)
		else:
			deploy_payloads(0, 0, 0)
