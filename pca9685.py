#!/usr/bin/env python

import time
import Adafruit_PCA9685
import rospy
from std_msgs.msg import String
bldc=Adafruit_PCA9685.PCA9685()

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

bldc.set_pwm_freq(60)        

'''
set_angle(0,100)
set_angle(1,100)
set_angle(2,100)
set_angle(3,100)
print('100')
time.sleep(10)

set_angle(0,0)
set_angle(1,0)
set_angle(2,0)
set_angle(3,0)
print('0')
time.sleep(5)
'''

if __name__ == '__main__':
    rospy.init_node('topic_sub_node')
    sub = rospy.Subscriber('/chatter',String,msg_callback,queue_size=1)
    
    rospy.spin()
