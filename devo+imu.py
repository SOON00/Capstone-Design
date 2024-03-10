#!/usr/bin/env python

import time
import Adafruit_PCA9685
import rospy
import math
from std_msgs.msg import String
import threading
import serial

roll,pitch,yaw=0.0,0.0,0.0

bldc=Adafruit_PCA9685.PCA9685()
ser = serial.Serial('/dev/ttyUSB1',115200)

servoMin=150
servoMax=550

def map(value,min_angle,max_angle,min_pulse,max_pulse):
    angle_range=max_angle-min_angle
    pulse_range=max_pulse-min_pulse
    scale_factor=float(angle_range)/float(pulse_range)
    return min_pulse+(value/scale_factor)
    
def set_angle(channel,angle):
    pulse=int(map(angle,0,1000,servoMin,servoMax))
    bldc.set_pwm(channel,0,pulse)
    
def msg_callback(msg):
    try:
        value = int(msg.data)
        value = int((value-1000))
        set_angle(0, value)
        set_angle(1, value)
        set_angle(2, value)
        set_angle(3, value)
        print(value)
    except ValueError:
        print("error")
        
def imu_data():
    while True:
        if ser.in_waiting > 0:
            received_data = ser.readline().decode().rstrip()
            received_data = received_data.replace("*","")
            split_data = received_data.split(',')
            roll,pitch,yaw = split_data[0],split_data[1],split_data[2]
            print(roll,pitch,yaw)

bldc.set_pwm_freq(60)        

if __name__ == '__main__':
    rospy.init_node('topic_sub_node')
    sub = rospy.Subscriber('/chatter',String,msg_callback,queue_size=1)
    imu_thread = threading.Thread(target=imu_data)
    imu_thread.start()
    imu_thread.join()

    
    rospy.spin()



