#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Int32, Float32MultiArray, MultiArrayLayout,MultiArrayDimension
import serial
import threading

roll,pitch,yaw=0.0,0.0,0.0
gX,gY,gZ=0.0,0.0,0.0

ser = serial.Serial('/dev/ttyUSB0',115200)

def pub_imu():
    pub=rospy.Publisher('imu',Float32MultiArray,queue_size = 1)     
    r = rospy.Rate(1000)
    roll,pitch,yaw=0.0,0.0,0.0
    gX,gY,gZ=0.0,0.0,0.0
    received_data="*0,0,0,0,0,0"
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            received_data = ser.readline().decode().rstrip()
            received_data = received_data.replace("*","")
            split_data = received_data.split(',')
            print(split_data)
            try:        
                roll,pitch,yaw=split_data[0],split_data[1],split_data[2]
                gX,gY,gZ=split_data[3],split_data[4],split_data[5]
                data_to_send = Float32MultiArray()
                array = [float(roll),float(pitch),float(yaw),float(gX),float(gY),float(gZ)]
                data_to_send.data=array
                pub.publish(data_to_send)
                r.sleep()

            except:
                print('error')
                continue

if __name__ == '__main__':
    rospy.init_node('imu_node')
    pub_imu()
    rospy.spin()
