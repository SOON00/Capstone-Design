#!/usr/bin/env python

import time
import Adafruit_PCA9685
import rospy
import math
from std_msgs.msg import String
bldc=Adafruit_PCA9685.PCA9685()
from sensor_msgs.msg import Imu

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
        
def imu_callback(msg):
    gx = msg.angular_velocity.x
    gy = msg.angular_velocity.y
    gz = msg.angular_velocity.z
    
    ax = msg.linear_acceleration.x
    ay = msg.linear_acceleration.y
    az = msg.linear_acceleration.z
    
    ARoll = (180/math.pi)*(math.atan(ax/(math.sqrt((ay*ay)+(az*az)))))
    APitch = (180/math.pi)*(math.atan(ay/(math.sqrt((ax*ax)+(az*az)))))
    
    print("roll",ARoll)
    print("pitch",APitch)

bldc.set_pwm_freq(60)        

if __name__ == '__main__':
    rospy.init_node('topic_sub_node')
    sub = rospy.Subscriber('/chatter',String,msg_callback,queue_size=1)
    sub2 = rospy.Subscriber('/camera/imu',Imu,imu_callback,queue_size=1)
    
    rospy.spin()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
