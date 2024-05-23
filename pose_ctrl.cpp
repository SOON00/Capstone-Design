#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>

float posX, posY, posZ;
float posX_d = 0, posY_d = 0, posZ_d = 0.2;

float kp = 1.0;
float roll_d_by_pos, pitch_d_by_pos, thrust_d_by_pos;
float limit = 1.0;

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

            pitch_d_by_pos = posX_d - posX;  // posX_d로 인해 발생
            roll_d_by_pos = posY_d - posY;   // posY_d로 인해 발생
            thrust_d_by_pos = posZ_d - posZ; // posZ_d로 인해 발생

            pose_cmd.x = kp * roll_d_by_pos;
            pose_cmd.y = kp * pitch_d_by_pos;
            pose_cmd.z = kp * thrust_d_by_pos;

            if (pose_cmd.x > limit) pose_cmd.x = limit;
            else if(pose_cmd.x < -limit) pose_cmd.x = -limit;
            if (pose_cmd.y > limit) pose_cmd.y = limit;
            else if(pose_cmd.y < -limit) pose_cmd.y = -limit;
            if (pose_cmd.z > 10) pose_cmd.z = 10;
            else if(pose_cmd.z < -10) pose_cmd.z = -10;

            pub.publish(pose_cmd);

            ROS_INFO("Drone Center Frame Position (x, y, z): (%.2f, %.2f, %.2f)", posX, posY, posZ);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}

