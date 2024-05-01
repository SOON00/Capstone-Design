#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <librealsense2/rs.hpp>
#include <array>

using namespace std;

rs2::pipeline initialize_camera() {
    rs2::pipeline p;
    rs2::config conf;
    conf.enable_stream(rs2_stream::RS2_STREAM_ACCEL);
    conf.enable_stream(rs2_stream::RS2_STREAM_GYRO);
    p.start(conf);
    return p;
}

array<double, 3> gyro_data(const rs2_vector& gyro) {
    return {gyro.x, gyro.y, gyro.z};
}

array<double, 3> accel_data(const rs2_vector& accel) {
    return {accel.x, accel.y, accel.z};
}

void publish_imu_data(rs2::pipeline& camera_pipe, int argc, char** argv) { // argc와 argv를 매개변수로 추가
    ros::init(argc, argv, "d455_imu_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
    ros::Rate rate(300);  // 400Hz

    while (ros::ok()) {
        rs2::frameset f = camera_pipe.wait_for_frames();
        auto accel_frame = f.first_or_default(rs2_stream::RS2_STREAM_ACCEL);
        auto gyro_frame = f.first_or_default(rs2_stream::RS2_STREAM_GYRO);

        rs2_vector accel = accel_frame.as<rs2::motion_frame>().get_motion_data();
        rs2_vector gyro = gyro_frame.as<rs2::motion_frame>().get_motion_data();

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";

        auto accel_data_array = accel_data(accel);
        auto gyro_data_array = gyro_data(gyro);

        imu_msg.linear_acceleration.x = -accel_data_array[2];
        imu_msg.linear_acceleration.y = -accel_data_array[0];
        imu_msg.linear_acceleration.z = -accel_data_array[1];

        imu_msg.angular_velocity.x = gyro_data_array[2];
        imu_msg.angular_velocity.y = -gyro_data_array[0];
        imu_msg.angular_velocity.z = -gyro_data_array[1];

        pub.publish(imu_msg);
        rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    rs2::pipeline camera_pipe;
    camera_pipe = initialize_camera();
    publish_imu_data(camera_pipe, argc, argv); // argc와 argv를 publish_imu_data 함수에 전달
    return 0;
}
