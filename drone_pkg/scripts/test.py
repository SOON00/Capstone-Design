#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_msgs.msg import TFMessage
import tf.transformations as tf_trans
import geometry_msgs.msg
import pyrealsense2 as rs

class AprilTagTFBroadcaster:
    def __init__(self):
        rospy.init_node('april_tag_tf_broadcaster', anonymous=True)
        
        self.bridge = CvBridge()
        self.tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)
        
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Use predefined dictionary for ArUco markers
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Initialize Realsense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        
    def __del__(self):
        # Stop Realsense pipeline
        self.pipeline.stop()
        
    def rgb_callback(self, rgb_image):
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            for i, marker_id in enumerate(ids):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, self.camera_matrix, self.dist_coeffs)
                
                rvec_matrix = cv2.Rodrigues(rvec)[0]
                tvec = tvec.squeeze()
                
                tf_translation = tf_trans.translation_matrix(tvec)
                tf_rotation = np.eye(4)
                tf_rotation[:3, :3] = rvec_matrix
                
                tf_pose = np.dot(tf_translation, tf_rotation)
                
                tf_msg = TFMessage()
                tf_transform = geometry_msgs.msg.TransformStamped()
                tf_transform.header.stamp = rospy.Time.now()
                tf_transform.header.frame_id = "base_link"
                tf_transform.child_frame_id = "april_tag_" + str(marker_id[0])
                tf_transform.transform.translation.x = tf_pose[0, 3]
                tf_transform.transform.translation.y = tf_pose[1, 3]
                tf_transform.transform.translation.z = tf_pose[2, 3]
                quaternion = tf_trans.quaternion_from_matrix(tf_pose)
                tf_transform.transform.rotation.x = quaternion[0]
                tf_transform.transform.rotation.y = quaternion[1]
                tf_transform.transform.rotation.z = quaternion[2]
                tf_transform.transform.rotation.w = quaternion[3]
                
                tf_msg.transforms.append(tf_transform)
                
                self.tf_pub.publish(tf_msg)

    def run(self):
        while not rospy.is_shutdown():
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
            
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            rgb_image = np.asanyarray(color_frame.get_data())
            
            # Process depth and RGB images here if needed
            
            # Convert RGB image to grayscale for AprilTag detection
            gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            
            # Detect AprilTags
            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.aruco_params)
            
            # Publish TF messages for detected AprilTags
            if ids is not None:
                self.rgb_callback(rgb_image)
            
            # Sleep briefly to control the rate of TF message publishing
            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        node = AprilTagTFBroadcaster()
        node.run()
    except rospy.ROSInterruptException:
        pass

