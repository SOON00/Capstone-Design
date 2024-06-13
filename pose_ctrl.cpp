#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

float posX, posY, posZ;
float posX_d = 0, posY_d = 0, posZ_d = 0.2;

float kp = 5.0; //3
float roll_d_by_pos, pitch_d_by_pos, thrust_d_by_pos;
float limit = 1; //0.2

ros::Publisher pub;
geometry_msgs::Vector3 pose_cmd;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_ctrl");
    ros::NodeHandle node;

    pub = node.advertise<geometry_msgs::Vector3>("/pose_cmd", 10);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(100);
    while (node.ok()) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("camera_odom_frame", "drone_center", ros::Time(0));
            posX = -transformStamped.transform.translation.x - 0.13;
            posY = -transformStamped.transform.translation.y;
            posZ = transformStamped.transform.translation.z - 0.12;

            // 쿼터니언에서 yaw 각도 추출
            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w
            );
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            yaw=-yaw;

            // 좌표 회전 변환
            float posX_rot = -(posX * cos(yaw) - posY * sin(yaw));
            float posY_rot = -(posX * sin(yaw) + posY * cos(yaw));

            pitch_d_by_pos = posX_d - posX_rot;  // 회전된 좌표 사용
            roll_d_by_pos = posY_d - posY_rot;   // 회전된 좌표 사용
            thrust_d_by_pos = posZ_d - posZ;     // posZ는 그대로 사용

            pose_cmd.x = kp * roll_d_by_pos;
            pose_cmd.y = kp * pitch_d_by_pos;
            pose_cmd.z = kp * thrust_d_by_pos;

            if (pose_cmd.x > limit) pose_cmd.x = limit;
            else if (pose_cmd.x < -limit) pose_cmd.x = -limit;
            if (pose_cmd.y > limit) pose_cmd.y = limit;
            else if (pose_cmd.y < -limit) pose_cmd.y = -limit;
            if (pose_cmd.z > 10) pose_cmd.z = 10;
            else if (pose_cmd.z < -10) pose_cmd.z = -10;

            pub.publish(pose_cmd);

            ROS_INFO("(x, y, z): (%.2f, %.2f, %.2f)", posX, posY, posZ);
            ROS_INFO("(x_rot, y_rot): (%.2f, %.2f)", posX_rot, posY_rot);
            ROS_INFO("yaw: %.2f", yaw);

        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}


