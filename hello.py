#!/usr/bin/env python

import time
import Adafruit_PCA9685
import rospy
import math
from std_msgs.msg import Int32
import threading
import serial

roll,pitch,yaw=0.0,0.0,0.0
gX,gY,gZ=0.0,0.0,0.0

#PID Variables
r_Iterm=0
r_stab_Iterm=0
roll_output=0

p_Iterm=0
p_stab_Iterm=0
pitch_output=0

def pid(pid_target,stabilize_kp,stabilize_ki,rate_kp,rate_ki,rpy_angle,gyro,stabilize_Iterm,rate_Iterm):
    angle_error = pid_target-float(rpy_angle)
    
    stabilize_Pterm=stabilize_kp*angle_error
    stabilize_Iterm+=stabilize_ki*angle_error*0.001
    
    desired_rate = stabilize_Pterm
    
    rate_error = desired_rate-float(gyro)
    
    rate_Pterm = rate_kp*rate_error
    rate_Iterm = rate_ki*rate_error*0.001

    output=int(rate_Pterm+rate_Iterm+stabilize_Iterm)
    
    return output,stabilize_Iterm,rate_Iterm



bldc=Adafruit_PCA9685.PCA9685()
ser = serial.Serial('/dev/ttyUSB1',115200)

servoMin=150
servoMax=550
def constrain_value(value):
    if value >= 900:
        return 900
    elif value <= 100:
        return 100
    else:
        return value
    
def map(value,min_angle,max_angle,min_pulse,max_pulse):
    angle_range=max_angle-min_angle
    pulse_range=max_pulse-min_pulse
    scale_factor=float(angle_range)/float(pulse_range)
    return min_pulse+(value/scale_factor)
    
def set_angle(channel,angle):
    pulse=int(map(angle,0,1000,servoMin,servoMax))
    bldc.set_pwm(channel,0,pulse)
    
motor1,motor2,motor3,motor4=0,0,0,0
def msg_callback(msg):
    global motor1,motor2,motor3,motor4
    try:
        value = int(msg.data)
        value = int((value-1000))
        motor1 = value+roll_output+pitch_output
        motor1 = constrain_value(motor1)
        motor2 = value+roll_output-pitch_output
        motor2 = constrain_value(motor2)
        motor3 = value-roll_output-pitch_output
        motor3 = constrain_value(motor3)
        motor4 = value-roll_output+pitch_output
        motor4 = constrain_value(motor4)
        
        set_angle(1, motor1)
        set_angle(2, motor2)
        set_angle(3, motor3)
        set_angle(0, motor4)
        
        print(value)
    except ValueError:
        print("error")
        
def imu_data():
    global roll_output,pitch_output,r_Iterm,r_stab_Iterm,p_Iterm,p_stab_Iterm
    global motor1,motor2,motor3,motor4
    while True:
        if ser.in_waiting > 0:
            received_data = ser.readline().decode().rstrip()
            received_data = received_data.replace("*","")
            split_data = received_data.split(',')
            print(split_data)
            try:        #pid(pid_target,stabilize_kp,stabilize_ki,rate_kp,rate_ki,rpy_angle,gyro,stabilize_Iterm,rate_Iterm):
                roll,pitch,yaw=split_data[0],split_data[1],split_data[2]
                gX,gY,gZ=split_data[3],split_data[4],split_data[5]
                roll_output,r_stab_Iterm,r_Iterm=pid(0,1,0,1,0,roll,gX,r_stab_Iterm,r_Iterm)
                pitch_output,p_stab_Iterm,p_Iterm=pid(0,1,0,1,0,pitch,gY,p_stab_Iterm,p_Iterm)
            except:
                continue
            #print(roll,pitch,yaw)

        

            

bldc.set_pwm_freq(60)        

if __name__ == '__main__':
    rospy.init_node('topic_sub_node')
    sub = rospy.Subscriber('/channel_5',Int32,msg_callback,queue_size=3)
    imu_thread = threading.Thread(target=imu_data)
    imu_thread.start()
    imu_thread.join()

    rospy.spin()
