#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt

# Initialize arrays for storing angular velocity data
x_data, y_data, z_data = [], [], []
time_data = []

# Define callback function for /imu/data topic
def imu_callback(msg):
    global x_data, y_data, z_data, time_data

    # Append the received angular velocities to the arrays
    x_data.append(msg.angular_velocity.x)
    y_data.append(msg.angular_velocity.y)
    z_data.append(msg.angular_velocity.z)
    time_data.append(rospy.get_time())

    # Keep only the last 1000 data points to prevent memory overflow
    if len(x_data) > 1000:
        x_data.pop(0)
        y_data.pop(0)
        z_data.pop(0)
        time_data.pop(0)

# Initialize the ROS node and subscribe to the topic
def listener():
    rospy.init_node('angular_velocity_plotter', anonymous=True)
    rospy.Subscriber("/imu_data", Imu, imu_callback)

    # Set up the real-time plot with three subplots
    plt.ion()  # Interactive mode on
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))  # Three subplots vertically stacked
    
    # Infinite loop to keep updating the plot
    while not rospy.is_shutdown():
        if len(time_data) > 1:
            ax1.clear()
            ax2.clear()
            ax3.clear()
            
            # Plot x, y, z angular velocities on separate subplots
            ax1.plot(time_data, x_data, label='X angular velocity', color='r')
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('X Angular Velocity (rad/s)')
            ax1.legend()
            ax1.set_title('IMU Angular Velocity - X')

            ax2.plot(time_data, y_data, label='Y angular velocity', color='g')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Y Angular Velocity (rad/s)')
            ax2.legend()
            ax2.set_title('IMU Angular Velocity - Y')

            ax3.plot(time_data, z_data, label='Z angular velocity', color='b')
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Z Angular Velocity (rad/s)')
            ax3.legend()
            ax3.set_title('IMU Angular Velocity - Z')

            plt.pause(0.01)  # Pause to update the plot

    plt.ioff()
    plt.show()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

