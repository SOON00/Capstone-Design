#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import pyrealsense2 as rs
import numpy as np

def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    prof = p.start(conf)
    return p

def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])

def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])

def publish_imu_data(camera_pipe):
    rospy.init_node('d455_imu_publisher', anonymous=True)
    pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    rate = rospy.Rate(200)  # 200Hz

    while not rospy.is_shutdown():
        f = camera_pipe.wait_for_frames()
        accel = accel_data(f[0].as_motion_frame().get_motion_data())
        gyro = gyro_data(f[1].as_motion_frame().get_motion_data())

        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        
        imu_msg.orientation_covariance = [-1]*9
        pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_pipe = initialize_camera()
        publish_imu_data(camera_pipe)
    except rospy.ROSInterruptException:
        pass
