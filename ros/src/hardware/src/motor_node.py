

import rospy
from std_msgs.msg import Bool, Int32

import serial
from time import sleep

try:
    ser = serial.Serial('/dev/ttyACM0', 9600)
except:
    ser = serial.Serial('/dev/ttyACM1', 9600)

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

def motorLateralCb(msg):
    print 'lateral cb', msg.data
    dir = msg.data

    # enable
    ser.write('7\r1\r')

    if dir == 1:
        ser.write('8\r1\r')
        ser.write('2\r0\r')
        ser.write('6\r70\r')
    elif dir == -1:
        ser.write('8\r0\r')
        ser.write('2\r1\r')
        ser.write('6\r70\r')
    else:
        ser.write('8\r0\r')
        ser.write('2\r0\r')
        ser.write('6\r0\r')


rospy.init_node('motor_node')

start_sub = rospy.Subscriber("/motor/start", Bool, motorStartCb)
lateral_sub = rospy.Subscriber("/motor/lateral2", Int32, motorLateralCb)
rospy.spin()