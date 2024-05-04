#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <librealsense2/rs.hpp>
#include <array>
#include <cmath>

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

// Madgwick filter parameters
constexpr double beta = 0.1;
double q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0; // quaternion elements representing the estimated orientation
double invSampleFreq = 1.0 / 280.0; // inverse sample frequency


void MadgwickAHRSupdateIMU(double ax, double ay, double az, double gx, double gy, double gz) {
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double hx, hy;
    double qDot1Half, qDot2Half, qDot3Half, qDot4Half;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
        // Normalise accelerometer measurement
        recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and magnetic field
        hx = 2.0 * q0 * az - 2.0 * q2 * ax;
        hy = 2.0 * q0 * ax + 2.0 * q1 * ay;

        // Rate of change of quaternion from gyroscope
        qDot1Half = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2Half = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        qDot3Half = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        qDot4Half = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        qDot1 -= beta * qDot1Half;
        qDot2 -= beta * qDot2Half;
        qDot3 -= beta * qDot3Half;
        qDot4 -= beta * qDot4Half;

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * invSampleFreq;
        q1 += qDot2 * invSampleFreq;
        q2 += qDot3 * invSampleFreq;
        q3 += qDot4 * invSampleFreq;

        // Normalise quaternion
        recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }
}

void publish_imu_data(rs2::pipeline& camera_pipe) {

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);
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

        // Call Madgwick filter to update quaternion
        MadgwickAHRSupdateIMU(accel_data_array[2], -accel_data_array[0], accel_data_array[1],
                              gyro_data_array[2],-gyro_data_array[0], -gyro_data_array[1]);

        // Convert quaternion to orientation
        imu_msg.orientation.w = q0;
        imu_msg.orientation.x = q1;
        imu_msg.orientation.y = q2;
        imu_msg.orientation.z = q3;

        imu_msg.linear_acceleration.x = accel_data_array[2];
        imu_msg.linear_acceleration.y = -accel_data_array[0];
        imu_msg.linear_acceleration.z = accel_data_array[1];

        imu_msg.angular_velocity.x = gyro_data_array[2];
        imu_msg.angular_velocity.y = -gyro_data_array[0];
        imu_msg.angular_velocity.z = -gyro_data_array[1];
        
        ROS_INFO("x:%lf, y:%lf, z:%lf w:%lf", q0, q1, q2, q3);

        pub.publish(imu_msg);
        rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "d455_imu_publisher");
    rs2::pipeline camera_pipe;
    camera_pipe = initialize_camera();
    publish_imu_data(camera_pipe);
    return 0;
}
