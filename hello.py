#!/usr/bin/env python

import mpu6050
import time
import Adafruit_PCA9685
import rospy
import math
from std_msgs.msg import String
import threading

bldc=Adafruit_PCA9685.PCA9685()
# Create a new Mpu6050 object
mpu6050 = mpu6050.mpu6050(0x68)

# Define a function to read the sensor data
def read_sensor_data():
    # Read the accelerometer values
    accelerometer_data = mpu6050.get_accel_data()

    # Read the gyroscope values
    gyroscope_data = mpu6050.get_gyro_data()

    # Read temp
    temperature = mpu6050.get_temp()

    return accelerometer_data, gyroscope_data, temperature
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
    lgx, lgy, lgz = 0, 0, 0
    GRoll, GPitch, GYaw = 0, 0, 0
    FRoll, FPitch = 0, 0
    while True:
        now = time.time()
        accelerometer_data, gyroscope_data, temperature = read_sensor_data()
        ax = accelerometer_data['x']
        ay = accelerometer_data['y']
        az = accelerometer_data['z']
        
        gx = gyroscope_data['x']
        gy = gyroscope_data['y']
        gz = gyroscope_data['z']
        
        ARoll = (180/math.pi)*(math.atan(ax/(math.sqrt((ay*ay)+(az*az)))))
        APitch = (180/math.pi)*(math.atan(ay/(math.sqrt((ax*ax)+(az*az)))))
        
        GRoll -= ((gy + lgy)/2)*0.03
        GPitch += ((gx + lgx)/2)*0.03
        GYaw += ((gz + lgz)/2)*0.03
        
        FRoll = 0.8*(FRoll-(((gy+lgy)/2)*0.03))+0.2*ARoll
        FPitch = 0.8*(FPitch-(((gx+lgx)/2)*0.03))+0.2*APitch
        
        lgx = gx
        lgy = gy
        lgz = gz
        
        #print("A :", ARoll)
        print("G :", GPitch)
        #print("Gy :", gy)
        print("A :", APitch)
        print("F :",FPitch)
        time.sleep(0.03-(time.time()-now))
    

bldc.set_pwm_freq(60)        

if __name__ == '__main__':
    rospy.init_node('topic_sub_node')
    sub = rospy.Subscriber('/chatter',String,msg_callback,queue_size=1)
    imu_thread = threading.Thread(target=imu_data)
    imu_thread.start()
    imu_thread.join()
    #sub2 = rospy.Subscriber('/camera/imu',Imu,imu_callback,queue_size=1)
    
    rospy.spin()



