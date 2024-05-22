#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

float posX, posY, posZ;

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_listener");
    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(100);
    while (node.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("camera_odom_frame", "drone_center", ros::Time(0));
            posX = -transformStamped.transform.translation.x;
            posY = -transformStamped.transform.translation.y;
            posZ = transformStamped.transform.translation.z;
            ROS_INFO("Drone Center Frame Position (x, y, z): (%.2f, %.2f, %.2f)", 
                     posX, posY, posZ);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
        }

        rate.sleep();
    }
    return 0;
}

