#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <librealsense2/rs.hpp>
#include <array>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;

class LowPassFilter {
private:
    double prev_value;
    double alpha;

public:
    LowPassFilter(double cutoff_freq, double dt) : prev_value(0.0) {
        // Alpha is the smoothing factor based on the cutoff frequency and sampling time (dt)
        double RC = 1.0 / (2.0 * M_PI * cutoff_freq);
        alpha = dt / (RC + dt);
    }

    double filter(double new_value) {
        prev_value = alpha * new_value + (1.0 - alpha) * prev_value;
        return prev_value;
    }
};
class LowPassFilter2ndOrder {
private:
    double prev_value1;
    double prev_value2;
    double alpha;

public:
    // Constructor to initialize the cutoff frequency and sampling time (dt)
    LowPassFilter2ndOrder(double cutoff_freq, double dt) 
        : prev_value1(0.0), prev_value2(0.0) {
        
        // Calculate RC and alpha (the smoothing factor)
        double RC = 1.0 / (2.0 * M_PI * cutoff_freq);
        alpha = dt / (RC + dt);
    }

    // Apply the 2nd order low-pass filter to the new input value
    double filter(double new_value) {
        // First order filter
        prev_value1 = alpha * new_value + (1.0 - alpha) * prev_value1;
        
        // Second order filter
        prev_value2 = alpha * prev_value1 + (1.0 - alpha) * prev_value2;

        return prev_value2;
    }
};

//LowPassFilter2ndOrder lpf_x(20, 0.005);  // cutoff frequency and 0.01s sampling time
LowPassFilter lpf_x(20, 0.005);
LowPassFilter lpf_y(20, 0.005);

const double PI = 3.1415926;
// 초기화 함수: RealSense D455 카메라 스트림 설정
rs2::pipeline initialize_camera() {
    rs2::pipeline p;
    rs2::config conf;
    conf.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 200);
    conf.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);
    p.start(conf);
    return p;
}

// 자이로스코프 데이터 추출
array<double, 3> gyro_data(const rs2_vector& gyro) {
    return {gyro.x, gyro.y, gyro.z};
}

// 가속도계 데이터 추출
array<double, 3> accel_data(const rs2_vector& accel) {
    return {accel.x, accel.y, accel.z};
}

// Madgwick 필터 파라미터
constexpr double beta = 0.15, zeta = 0.01;
double invSampleFreq = 1.0 / 200.0; // 샘플 주파수의 역수
double q10 = 1.0, q11 = 0.0, q12 = 0.0, q13 = 0.0;
double yaw_rotation_angle = 0.0;
int RC_arr[7];
double euler_rot = 0;
double real_rot = 0;
double x = 0;
double y = 318;

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
// PPM 데이터 콜백 함수 (사용 목적에 따라 수정 필요)
void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr &array){
    for(int i=0;i<7;i++){
        RC_arr[i]=array->data[i];
    }
    return;
}

// Madgwick 필터 업데이트 함수
void MadgwickQuaternionUpdate(double ax, double ay, double az, double gyrox, double gyroy, double gyroz)
{
    double q1 = q10, q2 = q11, q3 = q12, q4 = q13;         // 가독성을 위한 로컬 변수
    double norm;                                               // 벡터 노름
    double f1, f2, f3;                                         // 목적 함수 요소
    double J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // 목적 함수 야코비안 요소
    double qDot1, qDot2, qDot3, qDot4;
    double hatDot1, hatDot2, hatDot3, hatDot4;
    double gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // 자이로스코프 바이어스 에러

    // 반복 계산을 피하기 위한 보조 변수
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

    // 가속도계 측정값 정규화
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // NaN 처리
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    // 목적 함수와 야코비안 계산
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
  
    // 그래디언트 계산 (행렬 곱셈)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;
    
    // 그래디언트 정규화
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;
    
    // 자이로스코프 바이어스 에러 계산
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
    
    // 자이로스코프 바이어스 계산 및 제거
    gbiasx += gerrx * invSampleFreq * zeta;
    gbiasy += gerry * invSampleFreq * zeta;
    gbiasz += gerrz * invSampleFreq * zeta;
    gyrox -= gbiasx;
    gyroy -= gbiasy;
    gyroz -= gbiasz;
    
    // 쿼터니언 도함수 계산
    qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
    qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
    qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
    qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

    // 추정된 쿼터니언 도함수 적분
    q1 += (qDot1 - (beta * hatDot1)) * invSampleFreq;
    q2 += (qDot2 - (beta * hatDot2)) * invSampleFreq;
    q3 += (qDot3 - (beta * hatDot3)) * invSampleFreq;
    q4 += (qDot4 - (beta * hatDot4)) * invSampleFreq;

    // 쿼터니언 정규화
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q10 = q1 * norm;
    q11 = q2 * norm;
    q12 = q3 * norm;
    q13 = q4 * norm;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "d455_imu_publisher");
    rs2::pipeline camera_pipe;
    camera_pipe = initialize_camera();

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);
    ros::Subscriber devo = nh.subscribe("/PPM", 10, &arrayCallback);
    ros::Rate rate(200);

    // Z축 방향으로 45도 회전 (라디안 단위)

    while (ros::ok()) {
        rs2::frameset f = camera_pipe.wait_for_frames();
        auto accel_frame = f.first_or_default(rs2_stream::RS2_STREAM_ACCEL);
        auto gyro_frame = f.first_or_default(rs2_stream::RS2_STREAM_GYRO);

        rs2_vector accel = accel_frame.as<rs2::motion_frame>().get_motion_data();
        rs2_vector gyro = gyro_frame.as<rs2::motion_frame>().get_motion_data();

        sensor_msgs::Imu imu_msg;
        imu_msg.header.frame_id = "imu_link";

        auto accel_data_array = accel_data(accel);
        auto gyro_data_array = gyro_data(gyro);
        euler_rot = mapping(RC_arr[6],input_min,input_max,ang_min,ang_max);
        yaw_rotation_angle = -euler_rot*PI/180; // 45도
        
        y = 114.5 + 209*cos(yaw_rotation_angle);
        x = 209*sin(yaw_rotation_angle);
        
        real_rot = atan2(x,y);
        
        tf2::Matrix3x3 rotation_matrix;
        rotation_matrix.setRPY(0, 0, real_rot); // Roll, Pitch, Yaw

        // 회전 행렬을 사용하여 원시 데이터 회전
        tf2::Vector3 accel_vec(accel_data_array[2], -accel_data_array[0], -accel_data_array[1]);
        tf2::Vector3 gyro_vec(gyro_data_array[2], -gyro_data_array[0], -gyro_data_array[1]);

        tf2::Vector3 rotated_accel = rotation_matrix * accel_vec;
        tf2::Vector3 rotated_gyro = rotation_matrix * gyro_vec;

        // Madgwick 필터에 회전된 데이터 적용
        MadgwickQuaternionUpdate(rotated_accel.x(), rotated_accel.y(), rotated_accel.z(),
                                 rotated_gyro.x(), rotated_gyro.y(), rotated_gyro.z());

        // 쿼터니언 변환
        imu_msg.orientation.w = q10;
        imu_msg.orientation.x = q11;
        imu_msg.orientation.y = q12;
        imu_msg.orientation.z = q13;

        // 회전된 가속도 및 자이로스코프 데이터 IMU 메시지에 적용
        imu_msg.linear_acceleration.x = rotated_accel.x();
        imu_msg.linear_acceleration.y = rotated_accel.y();
        imu_msg.linear_acceleration.z = rotated_accel.z();


        imu_msg.angular_velocity.x = lpf_x.filter(rotated_gyro.x());
        imu_msg.angular_velocity.y = lpf_y.filter(rotated_gyro.y());
        //imu_msg.angular_velocity.x = rotated_gyro.x();
        //imu_msg.angular_velocity.y = rotated_gyro.y();
        imu_msg.angular_velocity.z = rotated_gyro.z();

        // 쿼터니언이 제대로 계산되었는지 확인을 위한 로그 (필요 시 활성화)
        //ROS_INFO("w:%lf", euler_rot);

        pub.publish(imu_msg);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

