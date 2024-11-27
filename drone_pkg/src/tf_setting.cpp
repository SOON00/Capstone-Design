#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>
#define PI 3.1415926

int RC_arr[7];
double euler_rot = 0;
double x = 0;
double y = 318;
double real_rot = 0;

int input_min = 944;
int input_max = 1833;
int ang_min = 45;
int ang_max = 0;

int mapping(double value,double min_pwm, double max_pwm,double min_pulse,double max_pulse){
    double pwm_range = max_pwm-min_pwm;//800
    double pulse_range = max_pulse-min_pulse;//410
    long double scale_factor = pwm_range/pulse_range;//800/410
    double result = min_pulse+((value-min_pwm)*pulse_range/pwm_range);//100+x*410/800
    return result;
}

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr &array) {
    for(int i=0;i<7;i++){
        RC_arr[i]=array->data[i];
    }
    euler_rot = mapping(RC_arr[6],input_min,input_max,ang_min,ang_max);
    euler_rot = -euler_rot*PI/180;
    y = 114.5 + 209*cos(euler_rot);
    x = 209*sin(euler_rot);
    real_rot = atan2(x,y);
    return;
}

void publishTransform(double angle_deg, tf2_ros::StaticTransformBroadcaster &static_broadcaster, geometry_msgs::TransformStamped &static_transformStamped) {
    tf2::Quaternion original_quat;
    original_quat.setRPY(0, 42 * PI / 180.0, PI);

    tf2::Quaternion additional_rotation;
    additional_rotation.setRPY(0, 0, -angle_deg);

    tf2::Quaternion combined_quat = original_quat * additional_rotation;
    combined_quat.normalize();

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.transform.rotation.x = combined_quat.x();
    static_transformStamped.transform.rotation.y = combined_quat.y();
    static_transformStamped.transform.rotation.z = combined_quat.z();
    static_transformStamped.transform.rotation.w = combined_quat.w();

    static_broadcaster.sendTransform(static_transformStamped);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_tf_broadcaster");
    ros::NodeHandle nh;

    // 초기 변환 설정
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.frame_id = "camera_link";
    static_transformStamped.child_frame_id = "drone_center";
    static_transformStamped.transform.translation.x = -0.17768;
    static_transformStamped.transform.translation.y = 0.0;
    static_transformStamped.transform.translation.z = 0.01189;

    // TF 브로드캐스터 초기화
    tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // 토픽 구독
    ros::Subscriber rotation_sub = nh.subscribe("/PPM", 10, &arrayCallback);

    ros::Rate rate(100); // 100 Hz
    while (ros::ok()) {
        // 변환을 주기적으로 게시
        publishTransform(real_rot, static_broadcaster, static_transformStamped);
        //ROS_INFO("w:%lf", real_rot);
        ros::spinOnce(); // 콜백 함수 호출
        rate.sleep();    // 설정한 주기만큼 대기
    }

    return 0;
}
/*
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#define PI 3.141592

int main(int argc, char **argv){
    ros::init(argc,argv,"setting_morph_tf");

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id="camera_link";
    static_transformStamped.child_frame_id="drone_center";
    static_transformStamped.transform.translation.x=-0.17768;
    static_transformStamped.transform.translation.y=0.0;
    static_transformStamped.transform.translation.z=0.01189;
    tf2::Quaternion quat;
    quat.setRPY(0,42*PI/180.0,PI);
    static_transformStamped.transform.rotation.x=quat.x();
    static_transformStamped.transform.rotation.y=quat.y();
    static_transformStamped.transform.rotation.z=quat.z();
    static_transformStamped.transform.rotation.w=quat.w();

    static_broadcaster.sendTransform(static_transformStamped);
    ros::spin();
    return 0;
}
*/
