#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt
import numpy as np

# Initialize arrays for storing linear velocity data
x_data, y_data, z_data = [], [], []
time_data = []

# Define callback function for /filtered_linear_velocity topic
def filtered_velocity_callback(msg):
    global x_data, y_data, z_data, time_data

    # Append the received velocities to the arrays
    x_data.append(msg.x)
    y_data.append(msg.y)
    z_data.append(msg.z)
    time_data.append(rospy.get_time())

    # Keep only the last 1000 data points to prevent memory overflow
    if len(x_data) > 1000:
        x_data.pop(0)
        y_data.pop(0)
        z_data.pop(0)
        time_data.pop(0)

# Initialize the ROS node and subscribe to the topic
def listener():
    rospy.init_node('velocity_plotter', anonymous=True)
    rospy.Subscriber("/filtered_linear_velocity", Vector3, filtered_velocity_callback)

    # Set up the real-time plot
    plt.ion()  # Interactive mode on
    fig, ax = plt.subplots()
    
    # Infinite loop to keep updating the plot
    while not rospy.is_shutdown():
        if len(time_data) > 1:
            ax.clear()
            
            # Plot x, y, z velocities
            ax.plot(time_data, x_data, label='X velocity', color='r')
            ax.plot(time_data, y_data, label='Y velocity', color='g')
            ax.plot(time_data, z_data, label='Z velocity', color='b')

            # Set plot labels and title
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Velocity (m/s)')
            ax.set_title('Filtered Linear Velocity')
            ax.legend()

            plt.pause(0.01)  # Pause to update the plot

    plt.ioff()
    plt.show()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

