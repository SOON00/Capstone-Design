#!/usr/bin/env python

import mpu6050
import time
import Adafruit_PCA9685
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Imu

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
        accelerometer_data, gyroscope_data, temperature = read_sensor_data()
        print("Accelerometer data:", accelerometer_data)
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
    accelerometer_data, gyroscope_data, temperature = read_sensor_data()
    print("Accelerometer data:", accelerometer_data)
    
    #ARoll = (180/math.pi)*(math.atan(ax/(math.sqrt((ay*ay)+(az*az)))))
    #APitch = (180/math.pi)*(math.atan(ay/(math.sqrt((ax*ax)+(az*az)))))
    

bldc.set_pwm_freq(60)        

if __name__ == '__main__':
    rospy.init_node('topic_sub_node')
    sub = rospy.Subscriber('/chatter',String,msg_callback,queue_size=1)
    #sub2 = rospy.Subscriber('/camera/imu',Imu,imu_callback,queue_size=1)
    
    rospy.spin()
    
# Start a while loop to continuously read the sensor data
'''
while True:

    # Read the sensor data
    accelerometer_data, gyroscope_data, temperature = read_sensor_data()

    # Print the sensor data
    print("Accelerometer data:", accelerometer_data)
    #print("Gyroscope data:", gyroscope_data)
    #print("Temp:", temperature)

    # Wait for 1 second
'''
