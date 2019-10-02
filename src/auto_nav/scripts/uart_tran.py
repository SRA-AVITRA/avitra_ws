#!/usr/bin/env python

###################################################################################################################
'''
CODE TO TRANSFER DATA BETWEEN ESP AND JETSON
RECEIVE:
    uint8_t motor_L.ticks_count
    uint8_t motor_R.ticks_count

    DECODING:
        if code & 0b10000000 == 1:



'''
###################################################################################################################
import serial
import struct
import rospy
from std_msgs.msg import String
import sys, select, termios, tty
from auto_nav.msg import uart_ticks_msg, uart_velocity_msg

# 425.16 ticks per (2*pi) radians
# ((pi/6)/(2*pi))*425.16 = 36 ticks for (pi/6)
rotation_step_size = 30
rot_flag = 0
rot_ticks_L = 0
rot_ticks_R = 0

#kP_initial = 0.0469
kP_initial = 0.0119
kD_initial = 20.0
kP = kP_initial
kD = kD_initial
kP_step = 0.005
kD_step = 1.0
def deploy_payloads(direction, speed):
    global motor_L, motor_R, rot_flag, rot_ticks_L, rot_ticks_R
    if direction == "w":
        motor_L = 41 + speed
        motor_R = 169 - speed #128 + 41
    elif direction == "s":
        motor_L = 41 - speed
        motor_R = 169 + speed #128 + 41
    elif direction == "a":
        rot_flag = 1
        rot_ticks_L = 0
        rot_ticks_R = 0
        motor_L = 41 - speed
        motor_R = 169 - speed #128 + 41
    elif direction == "d":
        rot_flag = 1
        rot_ticks_L = 0
        rot_ticks_R = 0
        motor_L = 41 + speed
        motor_R = 169 + speed #128 + 41
    else:
        motor_L = 41
        motor_R = 169
    ser.write([motor_L])
    ser.write([motor_R])
    print "*********",
    
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)                                         # read key pressed on keyboard
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    rospy.init_node('uart_tran',anonymous=False)
    try:
        ser = serial.Serial('/dev/ttyUSB0')
    except serial.serialutil.SerialException:
        ser = serial.Serial('/dev/ttyUSB1')
    ser.baudrate = 115200
    settings = termios.tcgetattr(sys.stdin)

    deploy_payloads("r", 0)
    direction = "r"
    speed = 0
    motor_L = 41
    motor_R = 169
    pub_ticks = rospy.Publisher('ticks', uart_ticks_msg, queue_size=10)          # publisher for teleop_key
    pub_velocity = rospy.Publisher("velocity", uart_velocity_msg, queue_size=10)
    rate = rospy.Rate(10)
    uart_ticks = uart_ticks_msg()
    uart_velocity = uart_velocity_msg()
    total_L = 0
    total_R = 0
    l_received_flag = 0
    r_received_flag = 0
    while not rospy.is_shutdown():
        while(ser.in_waiting):
            payload_c = ser.read()
            if payload_c != '':
                # payload = int(payload_c)
                payload = struct.unpack('>B', payload_c)[0]
                # print "payload_c = >", payload_c, "<\t\tpayload = >", payload, "<"
                if payload < 128:   #Left
                    uart_ticks.motor_L = payload - 41
                    total_L += uart_ticks.motor_L
                    uart_velocity.motor_L = uart_ticks.motor_L * 10
                    if rot_flag:
                        rot_ticks_L += uart_ticks.motor_L
                        if abs(rot_ticks_L) >= rotation_step_size:
                            rot_flag = 0
                            direction = "r"
                            deploy_payloads(direction, speed)
                    if r_received_flag:
                        pub_ticks.publish(uart_ticks)
                        pub_velocity.publish(uart_velocity)
                        r_received_flag = 0
                    else:
                        l_received_flag = 1
                else:   #Right
                    uart_ticks.motor_R = 169 - payload
                    total_R += uart_ticks.motor_R
                    uart_velocity.motor_R = uart_ticks.motor_R * 10
                    if rot_flag:
                        rot_ticks_R += uart_ticks.motor_R
                        if abs(rot_ticks_R) >= rotation_step_size:
                            rot_flag = 0
                            direction = "r"
                            deploy_payloads(direction, speed)
                    if l_received_flag:
                        pub_ticks.publish(uart_ticks)
                        pub_velocity.publish(uart_velocity)
                        l_received_flag = 0
                    else:
                        r_received_flag = 1
            # else:
            #     print "payload = >", payload, "<"
        print "direction = ", direction, "\tspeed = ", speed, "\ttotal L = ", total_L, "\tR = ", total_R, "rF: ", rot_flag, "rTL: ", rot_ticks_L, "rTR: ", rot_ticks_R
        key = getKey()
        if key == "r":
            direction = "r"
            deploy_payloads(direction, speed)
        elif key == "w":
            direction = "w"
            deploy_payloads(direction, speed)
        elif key == "s":
            direction = "s"
            deploy_payloads(direction, speed)
        elif key == "a":
            direction = "a"
            deploy_payloads(direction, speed)
        elif key == "d":
            direction = "d"
            deploy_payloads(direction, speed)
        elif key == "y":
            speed += 1
            if speed > 40:
                speed = 40
            if direction != "r":
                deploy_payloads(direction, speed)
        elif key == "h":
            speed -= 1
            if speed < 0:
                speed = 0
            if direction != "r":
                deploy_payloads(direction, speed)
        # elif key == "p":
        #     total_L = 0
        #     total_R = 0
            # kP = kP_initial
            # kD = kD_initial
        elif key == "i":
            kP += kP_step
            ser.write([101])
            print "kP increased ++, kP = ", kP, "\tkD = ", kD
        elif key == "k":
            kP -= kP_step
            ser.write([100])
            print "kP decreased --, kP = ", kP, "\tkD = ", kD
        elif key == "o":
            kD += kD_step
            ser.write([103])
            print "kD increased ++, kP = ", kP, "\tkD = ", kD
        elif key == "l":
            kD -= kD_step
            ser.write([102])
            print "kD decreased --, kP = ", kP, "\tkD = ", kD
        elif key == "p":
            # ser.write([105])
            print "kP = ", kP, "\tkD = ", kD
        elif key == "x":
            deploy_payloads("r", 0)
            print "\nEND\t key = >", key, "< pressed\tspeed = ", speed, "\tkey = ", key, "\tmotor_L = ", motor_L, "\t motor_R =- ", motor_R
            exit()
        elif key != "":
            direction = "r"
            deploy_payloads("r", 0)
            print "\nKILL\t key = >", key, "< pressed\tspeed = ", speed, "\tkey = ", key, "\tmotor_L = ", motor_L, "\t motor_R =- ", motor_R
        if key != "":
            print "speed = ", speed, "\tkey = ", key, "\tmotor_L = ", motor_L, "\t motor_R = ", motor_R
