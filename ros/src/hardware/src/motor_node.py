

import rospy
from std_msgs.msg import Bool

import serial
from time import sleep

ser = serial.Serial('/dev/ttyACM0', 9600)

def motorStartCb(msg):
    print 'start cb', msg.data

    if msg.data:
        ser.write('3\r')
        ser.write('50\r')
        ser.write('5\r')
        ser.write('50\r')
    else:
        ser.write('3\r')
        ser.write('0\r')
        ser.write('5\r')
        ser.write('0\r')


rospy.init_node('motor_node')

start_sub = rospy.Subscriber("/motor/start", Bool, motorStartCb)
rospy.spin()