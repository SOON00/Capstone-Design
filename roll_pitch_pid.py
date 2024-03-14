#!/usr/bin/env python

import time
import Adafruit_PCA9685
import rospy
import math
from std_msgs.msg import String
import threading
import serial

roll,pitch,yaw=0.0,0.0,0.0

#PID Variables
Iterm=0
prev_Dterm=0
roll_output=0

p_Iterm=0
p_prev_Dterm=0
pitch_output=0

def pid(pid_target,pid_kp,pid_ki,pid_kd,rpy_angle,Iterm,prev_Dterm):
    rate_kp=pid_kp
    rate_ki=pid_ki
    rate_kd=pid_kd

    error = pid_target-float(rpy_angle)
    Pterm=rate_kp*error
    Iterm+=rate_ki*error*0.001
    Dterm=-((float(rpy_angle) - prev_Dterm)/0.001)*rate_kd
    Dterm=prev_Dterm*0.9+Dterm*0.1

    output=int(Pterm+Iterm+Dterm)
    
    prev_target=float(rpy_angle)
    prev_Dterm=Dterm
    
    return output,Iterm,prev_Dterm



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
        set_angle(1, value+roll_output+pitch_output)
        set_angle(2, value+roll_output-pitch_output)
        set_angle(3, value-roll_output-pitch_output)
        set_angle(0, value-roll_output+pitch_output)
        print(value-roll_output-pitch_output)
    except ValueError:
        print("error")
        
def imu_data():
    global roll_output,pitch_output,Iterm,prev_Dterm,p_Iterm,p_prev_Dterm
    #global roll, pitch, yaw
    #global Iterm, prev_Dterm,roll, pitch, yaw
    while True:
        if ser.in_waiting > 0:
            received_data = ser.readline().decode().rstrip()
            received_data = received_data.replace("*","")
            split_data = received_data.split(',')
            #print(split_data)
            try:
                roll,pitch,yaw=split_data[0],split_data[1],split_data[2]
                roll_output,Iterm,prev_Dterm=pid(0,0.5,0,0,roll,Iterm,prev_Dterm)
                pitch_output,p_Iterm,p_prev_Dterm=pid(0,0.5,0,0,pitch,p_Iterm,p_prev_Dterm)
            except:
                continue
            #print(roll,pitch,yaw)

bldc.set_pwm_freq(60)        

if __name__ == '__main__':
    rospy.init_node('topic_sub_node')
    sub = rospy.Subscriber('/chatter',String,msg_callback,queue_size=1)
    imu_thread = threading.Thread(target=imu_data)
    imu_thread.start()
    imu_thread.join()

    rospy.spin()



