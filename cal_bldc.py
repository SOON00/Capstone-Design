#!/usr/bin/env python

import time
import rospy
from rospy.numpy_msg import numpy_msg
import Adafruit_PCA9685
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayLayout,MultiArrayDimension
from rospy.numpy_msg import numpy_msg

bldc=Adafruit_PCA9685.PCA9685()

servoMin=1640
servoMax=3200
def constrain(x):
    if x>=1800:
        x=1800
    elif x<=200:
        x=200
    return x
def map(value,min_angle,max_angle,min_pulse,max_pulse):
    angle_range=max_angle-min_angle
    pulse_range=max_pulse-min_pulse
    scale_factor=float(angle_range)/float(pulse_range)
    return min_pulse+(value/scale_factor)
    
def set_angle(channel,angle):
    pulse=int(map(angle,200,1800,servoMin,servoMax))
    bldc.set_pwm(channel,0,pulse)

def msg_callback(msg):
    try:
        value = Float32MultiArray()
        value = msg.data
        
        
        motor1 = 2*(value[3]-1000)
        motor2 = 2*(value[3]-1000)
        motor3 = 2*(value[3]-1000)
        motor4 = 2*(value[3]-1000)
        motor1=constrain(motor1)
        motor2=constrain(motor2)
        motor3=constrain(motor3)
        motor4=constrain(motor4)
        
        set_angle(1, motor1)
        set_angle(2, motor2)
        set_angle(3, motor3)
        set_angle(0, motor4) 
        print(motor1)
    except ValueError:
        print("error")

bldc.set_pwm_freq(400)

if __name__ == '__main__':
    rospy.init_node('array_sub')
    sub = rospy.Subscriber('/PPM',Float32MultiArray,msg_callback,queue_size=10)
    rospy.spin()
