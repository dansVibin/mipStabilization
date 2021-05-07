#!/usr/bin/python3
import serial
import rospy
from std_msgs.msg import Int32
rospy.init_node('pot_publisher')
pub = rospy.Publisher('/pot_Position',Int32,queue_size = 1)
rate = rospy.Rate(500)

#connect to arduino serial monitor
ser = serial.Serial('/dev/ttyACM0', 115200)
while not rospy.is_shutdown():
    pos = ser.readline()
    data = pos.decode('utf-8') #decode text data
    pub.publish(int(data))
    rate.sleep()

