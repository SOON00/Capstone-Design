#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <nav_msgs/Odometry.h>

class PIDController{
private:
    double Kp;
    double Ki;
    double Kd;

    double integral;
    double pre_error;
public:
    PIDController(double p, double i, double d) :
    Kp(p), Ki(i), Kd(d), integral(0), pre_error(0) {}
    double calculate(double target, double input, double dt, double target_rate){
        double error = target - input;
        
        double P_term = Kp*error;
        
        integral += error*dt;
        double I_term = Ki * integral;
        
        double derivative = target_rate;
        double D_term = Kd * derivative;
        
        double controlOutput = P_term + I_term + D_term;
        
        pre_error = error;
        
        return controlOutput; 
    }
};

volatile double linear_velocity_x = 0;
volatile double linear_velocity_y = 0;
double linear_velocity_z = 0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

    double linear_v_x = msg->twist.twist.linear.x;
    double linear_v_y = msg->twist.twist.linear.y;
    double linear_v_z = msg->twist.twist.linear.z;
    linear_velocity_x = linear_v_x;
    linear_velocity_y = linear_v_y;
    linear_velocity_z = linear_v_z;

}

PIDController desired_X(0.05,0,0.005);

PIDController desired_X_v(1,0,0);
float kp_l_v = 1;

PIDController desired_Y(0,0,0);
PIDController desired_Z(2,0,1);

float posX, posY, posZ;
float posX_d = 0, posY_d = 0, posZ_d = 0.2;

float kp = 5.0; //3
float limit = 0.05; //0.2
float l_v_limit = 1; //0.2

ros::Publisher pub;
ros::Publisher t265_yaw;
geometry_msgs::Vector3 pose_cmd;
std_msgs::Float32 yaw_cmd;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_ctrl");
    ros::NodeHandle node;

    pub = node.advertise<geometry_msgs::Vector3>("/pose_cmd", 10);
    t265_yaw = node.advertise<std_msgs::Float32>("/yaw_cmd", 10);
    
    ros::Subscriber odom_sub = node.subscribe("/camera/odom/sample", 1000, odomCallback);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(200);
    while (node.ok()) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("new_origin", "drone_center", ros::Time(0));
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
            yaw_cmd.data = yaw;
            yaw=-yaw;
            
            // 좌표 회전 변환
            float posX_rot = -(posX * cos(yaw) - posY * sin(yaw));
            float posY_rot = -(posX * sin(yaw) + posY * cos(yaw));
            
            //pose_cmd.x = desired_X.calculate(posY_d, posY_rot, 0.005,linear_velocity_y);
            
            double target_l_v = kp_l_v * (posY_d-posY_rot);
            
            if (target_l_v > l_v_limit) target_l_v = l_v_limit;
            else if (target_l_v < -l_v_limit) target_l_v = -l_v_limit;
            
            pose_cmd.x = desired_X_v.calculate(0, -linear_velocity_y, 0.005,0);//target_l_v
            
            pose_cmd.y = desired_Y.calculate(posX_d, posX_rot, 0.005,linear_velocity_x);
            pose_cmd.z = desired_Z.calculate(posZ_d, posZ, 0.005,0);

            //if (pose_cmd.x > limit) pose_cmd.x = limit;
            //else if (pose_cmd.x < -limit) pose_cmd.x = -limit;
            if (pose_cmd.y > limit) pose_cmd.y = limit;
            else if (pose_cmd.y < -limit) pose_cmd.y = -limit;
            if (pose_cmd.z > 10) pose_cmd.z = 10;
            else if (pose_cmd.z < -10) pose_cmd.z = -10;

            pub.publish(pose_cmd);
            t265_yaw.publish(yaw_cmd);

            //ROS_INFO("(x, y, z): (%.2f, %.2f, %.2f)", posX, posY, posZ);
            //ROS_INFO("(x_rot, y_rot): (%.2f, %.2f)", posX_rot, posY_rot);
            //ROS_INFO("linear y: %.2f", linear_velocity_y);

        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

