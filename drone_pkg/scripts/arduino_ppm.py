#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Float32MultiArray
import serial
import numpy as np
from collections import deque

# 버퍼 크기 설정
BUFFER_SIZE = 5

# 각 센서에 대한 버퍼 생성
buffers = {
    'roll': deque(maxlen=BUFFER_SIZE),
    'pitch': deque(maxlen=BUFFER_SIZE),
    'yaw': deque(maxlen=BUFFER_SIZE),
    'gX': deque(maxlen=BUFFER_SIZE),
    'gY': deque(maxlen=BUFFER_SIZE),
    'gZ': deque(maxlen=BUFFER_SIZE),
    'arr': deque(maxlen=BUFFER_SIZE)
}

def connect_serial():
    while not rospy.is_shutdown():
        try:
            ser = serial.Serial('/dev/ttyACM0', 9600)
            print("Serial connection established.")
            return ser
        except serial.SerialException:
            print("Failed to connect to /dev/ttyPPM. Retrying in 1 seconds...")
            time.sleep(1)
    return None

def apply_median_filter(data_dict):
    filtered_data = []
    for key, buffer in data_dict.items():
        if len(buffer) == 0:  # 버퍼가 비어 있는 경우를 처리
            median_value = 0
        else:
            median_value = np.median(list(buffer))
        filtered_data.append(median_value)
    return filtered_data

def pub_imu():
    pub = rospy.Publisher('PPM', Float32MultiArray, queue_size=10)
    r = rospy.Rate(200)
    ser = connect_serial()

    if ser is None:  # rospy is shutting down
        return

    while not rospy.is_shutdown():
        try:
            if ser.in_waiting > 0:
                try:
                    received_data = ser.readline().decode('utf-8', errors='ignore').rstrip()
                    received_data = received_data.replace("*", "")
                    split_data = received_data.split()

                    roll, pitch, yaw = split_data[0], split_data[1], split_data[2]
                    gX, gY, gZ = split_data[3], split_data[4], split_data[5]
                    arr = split_data[6]

                    # 버퍼에 데이터 추가
                    buffers['roll'].append(float(roll))
                    buffers['pitch'].append(float(pitch))
                    buffers['yaw'].append(float(yaw))
                    buffers['gX'].append(float(gX))
                    buffers['gY'].append(float(gY))
                    buffers['gZ'].append(float(gZ))
                    buffers['arr'].append(float(arr))

                    # 중간값 필터 적용
                    filtered_values = apply_median_filter(buffers)

                    data_to_send = Float32MultiArray()
                    data_to_send.data = filtered_values
                    pub.publish(data_to_send)
                    r.sleep()

                except (ValueError, IndexError) as e:
                    print(f'Error processing data: {e}')
                    continue

        except (serial.SerialException, OSError) as e:
            print(f"Serial connection lost: {e}. Reconnecting...")
            ser = connect_serial()
            if ser is None:
                break  # rospy is shutting down

if __name__ == '__main__':
    rospy.init_node('PPM_node')
    try:
        pub_imu()
    except rospy.ROSInterruptException:
        pass

