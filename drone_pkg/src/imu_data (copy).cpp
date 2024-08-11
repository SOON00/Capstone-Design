#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <librealsense2/rs.hpp>
#include <array>
#include <cmath>
#include <thread>
#include <mutex>

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
constexpr double beta = 0.15, zeta = 0.01;
double invSampleFreq = 1.0 / 300.0; // inverse sample frequency
double q10 = 1.0, q11 = 0.0, q12 = 0.0, q13 = 0.0;
void MadgwickQuaternionUpdate(double ax, double ay, double az, double gyrox, double gyroy, double gyroz)
        {
            double q1 = q10, q2 = q11, q3 = q12, q4 = q13;         // short name local variable for readability
            double norm;                                               // vector norm
            double f1, f2, f3;                                         // objetive funcyion elements
            double J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
            double qDot1, qDot2, qDot3, qDot4;
            double hatDot1, hatDot2, hatDot3, hatDot4;
            double gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

            // Auxiliary variables to avoid repeated arithmetic
            double _halfq1 = 0.5f * q1;
            double _halfq2 = 0.5f * q2;
            double _halfq3 = 0.5f * q3;
            double _halfq4 = 0.5f * q4;
            double _2q1 = 2.0f * q1;
            double _2q2 = 2.0f * q2;
            double _2q3 = 2.0f * q3;
            double _2q4 = 2.0f * q4;
            double _2q1q3 = 2.0f * q1 * q3;
            double _2q3q4 = 2.0f * q3 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;
            
            // Compute the objective function and Jacobian
            f1 = _2q2 * q4 - _2q1 * q3 - ax;
            f2 = _2q1 * q2 + _2q3 * q4 - ay;
            f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
            J_11or24 = _2q3;
            J_12or23 = _2q4;
            J_13or22 = _2q1;
            J_14or21 = _2q2;
            J_32 = 2.0f * J_14or21;
            J_33 = 2.0f * J_11or24;
          
            // Compute the gradient (matrix multiplication)
            hatDot1 = J_14or21 * f2 - J_11or24 * f1;
            hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
            hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
            hatDot4 = J_14or21 * f1 + J_11or24 * f2;
            
            // Normalize the gradient
            norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
            hatDot1 /= norm;
            hatDot2 /= norm;
            hatDot3 /= norm;
            hatDot4 /= norm;
            
            // Compute estimated gyroscope biases
            gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
            gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
            gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
            
            // Compute and remove gyroscope biases
            gbiasx += gerrx * invSampleFreq * zeta;
            gbiasy += gerry * invSampleFreq * zeta;
            gbiasz += gerrz * invSampleFreq * zeta;
            gyrox -= gbiasx;
            gyroy -= gbiasy;
            gyroz -= gbiasz;
            
            // Compute the quaternion derivative
            qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
            qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
            qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
            qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

            // Compute then integrate estimated quaternion derivative
            q1 += (qDot1 -(beta * hatDot1)) * invSampleFreq;
            q2 += (qDot2 -(beta * hatDot2)) * invSampleFreq;
            q3 += (qDot3 -(beta * hatDot3)) * invSampleFreq;
            q4 += (qDot4 -(beta * hatDot4)) * invSampleFreq;

            // Normalize the quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q10 = q1 * norm;
            q11 = q2 * norm;
            q12 = q3 * norm;
            q13 = q4 * norm;
        }


std::mutex mtx;
rs2_vector latest_accel, latest_gyro;

void sensor_data_acquisition(rs2::pipeline& camera_pipe) {
    while (ros::ok()) {
        rs2::frameset f = camera_pipe.wait_for_frames();
        auto accel_frame = f.first_or_default(rs2_stream::RS2_STREAM_ACCEL);
        auto gyro_frame = f.first_or_default(rs2_stream::RS2_STREAM_GYRO);

        std::lock_guard<std::mutex> lock(mtx);
        latest_accel = accel_frame.as<rs2::motion_frame>().get_motion_data();
        latest_gyro = gyro_frame.as<rs2::motion_frame>().get_motion_data();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "d455_imu_publisher");
    rs2::pipeline camera_pipe = initialize_camera();
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);
    ros::Rate rate(600);

    std::thread sensor_thread(sensor_data_acquisition, std::ref(camera_pipe));

    while (ros::ok()) {
        rs2_vector accel, gyro;
        {
            std::lock_guard<std::mutex> lock(mtx);
            accel = latest_accel;
            gyro = latest_gyro;
        }

        sensor_msgs::Imu imu_msg;
        imu_msg.header.frame_id = "imu_link";

        auto accel_data_array = accel_data(accel);
        auto gyro_data_array = gyro_data(gyro);

        // Call Madgwick filter to update quaternion
        MadgwickQuaternionUpdate(accel_data_array[2], -accel_data_array[0], -accel_data_array[1],
                              gyro_data_array[2],-gyro_data_array[0], -gyro_data_array[1]);

        // Convert quaternion to orientation
        imu_msg.orientation.w = q10;
        imu_msg.orientation.x = q11;
        imu_msg.orientation.y = q12;
        imu_msg.orientation.z = q13;

        imu_msg.linear_acceleration.x = accel_data_array[2];
        imu_msg.linear_acceleration.y = -accel_data_array[0];
        imu_msg.linear_acceleration.z = accel_data_array[1];

        imu_msg.angular_velocity.x = gyro_data_array[2];
        imu_msg.angular_velocity.y = -gyro_data_array[0];
        imu_msg.angular_velocity.z = -gyro_data_array[1];
        
        pub.publish(imu_msg);
        rate.sleep();
        ros::spinOnce();
    }

    sensor_thread.join();
    return 0;
}




