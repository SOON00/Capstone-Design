#!/usr/bin/env python

import time
import rospy
from rospy.numpy_msg import numpy_msg
import Adafruit_PCA9685
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayLayout,MultiArrayDimension
from rospy.numpy_msg import numpy_msg

bldc=Adafruit_PCA9685.PCA9685()

servoMin=150
servoMax=550
def constrain(x):
    if x>=900:
        x=900
    elif x<=100:
        x=100
    return x
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
        value = Float32MultiArray()
        value = msg.data
        
        motor1 = value[4]-1000
        motor2 = value[4]-1000
        motor3 = value[4]-1000
        motor4 = value[4]-1000
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

bldc.set_pwm_freq(60)

if __name__ == '__main__':
    rospy.init_node('array_sub')
    sub = rospy.Subscriber('/PPM',Float32MultiArray,msg_callback,queue_size=10)
    rospy.spin()
