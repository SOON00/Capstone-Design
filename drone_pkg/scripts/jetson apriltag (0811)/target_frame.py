import rospy
import pyrealsense2 as rs
import numpy as np
import cv2
import apriltag
import transforms3d
import tf
from geometry_msgs.msg import Pose

class AprilTagTfBroadcaster:
    def __init__(self):
        rospy.init_node('apriltag_tf_broadcaster', anonymous=True)
    
        self.pose_publisher = rospy.Publisher('pose', Pose, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()  # TF 리스너 초기화
    
        # RealSense 카메라 초기화
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
    
        # 카메라 내부 파라미터 얻기
        profile = self.pipeline.get_active_profile()
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.cmtx = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        self.dist = np.array(intr.coeffs)
    
        # AprilTag 검출기 설정
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))

        # 타이머 설정
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def detect_apriltags(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
        
        if len(detections) > 0:
            positions = []
            orientations = []
            for detection in detections:
                # 꼭지점 좌표 추출
                corners = np.array(detection.corners)
                
                # Convert to float32
                corners = corners.astype(np.float32)
                
                # Define 3D points of the tag's corners
                tag_size = 0.065  # Size of the tag in meters
                tag_corners_3d = np.array([
                    [-tag_size / 2, -tag_size / 2, 0],
                    [tag_size / 2, -tag_size / 2, 0],
                    [tag_size / 2, tag_size / 2, 0],
                    [-tag_size / 2, tag_size / 2, 0]
                ], dtype='float32')
                
                # Solve PnP to find the tag's pose using a more robust method
                ret, rvec, tvec = cv2.solvePnP(tag_corners_3d, corners, self.cmtx, self.dist, flags=cv2.SOLVEPNP_IPPE)
                if ret:
                    # Convert rotation vector to rotation matrix
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    # Convert rotation matrix to quaternion
                    quat = transforms3d.quaternions.mat2quat(rotation_matrix)

                    # Store positions and orientations
                    positions.append(tvec)
                    orientations.append(quat)

                    # Broadcast the transform
                    self.tf_broadcaster.sendTransform(
                        (tvec[0][0], tvec[1][0], tvec[2][0]),
                        (quat[1], quat[2], quat[3], quat[0]),
                        rospy.Time.now(),
                        f'apriltag_{detection.tag_id}',
                        'camera_link'
                    )

                    # Draw detected tag and its orientation on the image
                    self.draw_on_image(image, corners, tvec, rotation_matrix)

            if positions and orientations:
                self.compute_and_broadcast_target_frame(positions, orientations)

    def compute_and_broadcast_target_frame(self, positions, orientations):
        # Compute the average position
        avg_position = np.mean(positions, axis=0).flatten()

        # Compute the average orientation (quaternion)
        avg_orientation = np.mean(orientations, axis=0)
        avg_orientation = avg_orientation / np.linalg.norm(avg_orientation)  # Normalize quaternion

        # Convert average orientation to rotation matrix
        rotation_matrix = transforms3d.quaternions.quat2mat(avg_orientation)

        # Define the translation vector in the target_frame's local Z direction
        translation_in_target_frame = np.array([0, 0, -0.5])

        # Transform this translation vector to the camera_link frame
        adjusted_position = avg_position + np.dot(rotation_matrix, translation_in_target_frame)

        # Broadcast the adjusted target_frame
        self.tf_broadcaster.sendTransform(
            (adjusted_position[0], adjusted_position[1], adjusted_position[2]),
            (avg_orientation[1], avg_orientation[2], avg_orientation[3], avg_orientation[0]),
            rospy.Time.now(),
            'target_frame',
            'camera_link'
        )

    def draw_on_image(self, image, corners, tvec, rotation_matrix):
        # Draw detected AprilTag
        corners = corners.astype(int)
        cv2.polylines(image, [corners], True, (0, 255, 0), 2)
        center = np.mean(corners, axis=0).astype(int)
        cv2.circle(image, tuple(center), 5, (0, 0, 255), -1)

        # Draw orientation axes
        axis_length = 0.1  # Length of the axis in meters
        origin = (center[0], center[1])
        
        # Define the 3D axes
        axes = {
            'x': np.array([axis_length, 0, 0]),
            'y': np.array([0, axis_length, 0]),
            'z': np.array([0, 0, axis_length])
        }
        
        # Project 3D axes to 2D
        for axis_name, axis_vec in axes.items():
            axis_end = np.dot(rotation_matrix, axis_vec) + np.array([tvec[0][0], tvec[1][0], tvec[2][0]])
            axis_end = np.dot(self.cmtx, axis_end[:3] / axis_end[2])
            axis_end = (int(axis_end[0]), int(axis_end[1]))
            
            color = {'x': (0, 0, 255), 'y': (0, 255, 0), 'z': (255, 0, 0)}[axis_name]
            cv2.line(image, origin, axis_end, color, 2)
            cv2.putText(image, axis_name, (axis_end[0], axis_end[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    def get_camera_position(self):
        max_attempts = 5  # 최대 시도 횟수
        for attempt in range(max_attempts):
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/target_frame', '/camera_link', rospy.Time(0))
                return trans, rot

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr("TF Error: %s", str(e))
                rospy.sleep(0.1)  # 잠시 대기 후 다시 시도
                continue

        rospy.logerr("Failed to get TF after %d attempts", max_attempts)
        return None, None

    def timer_callback(self, event):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=10000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return
            
            img = np.asanyarray(color_frame.get_data())
            
            # Detect AprilTags and compute transforms
            self.detect_apriltags(img)

            # 카메라의 위치를 가져옵니다
            self.get_camera_position()

            # Display the image using OpenCV
            cv2.imshow('RGB with AprilTag', img)
            cv2.waitKey(1)

        except RuntimeError as e:
            rospy.logerr("RuntimeError in wait_for_frames: %s", str(e))
            self.pipeline.stop()
            self.pipeline.start()  # Restart the pipeline after error

def main():
    apriltag_tf_broadcaster = AprilTagTfBroadcaster()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

