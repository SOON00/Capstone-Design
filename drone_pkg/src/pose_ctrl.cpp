#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <nav_msgs/Odometry.h>

class dualPIDController{
private:
    double angle_Kp;
    double angle_Ki;
    double angle_Kd;

    double rate_Kp;
    double rate_Ki;
    double rate_Kd;
    
    double prevError;
    double angle_integral;
    double rate_integral;
    
    double i_limit;
    
    double prev_rate_D_term;
public:
    dualPIDController(double angle_p, double angle_i, double angle_d, double rate_p, double rate_i, double rate_d) :
    angle_Kp(angle_p), angle_Ki(angle_i), angle_Kd(angle_d), rate_Kp(rate_p), rate_Ki(rate_i), rate_Kd(rate_d), prevError(0), angle_integral(0), rate_integral(0), i_limit(0.1),prev_rate_D_term(0) {}
    double calculate(double target, double angle_input, double rate_input, double dt){
        double angle_error = target - angle_input;
        
        double angle_P_term = angle_Kp*angle_error;
        
        angle_integral += angle_error*dt;
        if(fabs(angle_integral)>i_limit)	angle_integral=(angle_integral/fabs(angle_integral))*i_limit;
        double angle_I_term = angle_Ki * angle_integral;
        
        double angle_D_term = angle_Kd * rate_input;
        
        double target_rate = angle_P_term + angle_I_term + angle_D_term;
        
        //if(target_rate>0.15) target_rate = 0.15;
        //else if(target_rate<-0.15) target_rate = -0.15;
        
        double rate_error = target_rate - rate_input;
        
        double rate_P_term = rate_Kp*rate_error;
        
        rate_integral += rate_error*dt;
        if(fabs(rate_integral)>i_limit)	rate_integral=(rate_integral/fabs(rate_integral))*i_limit;
        double rate_I_term = rate_Ki * rate_integral;
        
        double rate_D_term = (rate_error-prevError)/dt;
        
        rate_D_term = rate_Kd * (0.95 * prev_rate_D_term + 0.05 * rate_D_term);
        
        prev_rate_D_term = rate_D_term;
        
        prevError = rate_error;
        
        double controlOutput = rate_P_term + rate_I_term + rate_D_term;
        
        return controlOutput; 
    }
};//dualPID control class

class PIDController{
private:
    double Kp;
    double Ki;
    double Kd;

    double integral;
    double pre_error;
    double i_error_limit;
public:
    PIDController(double p, double i, double d) :
    Kp(p), Ki(i), Kd(d), integral(0), pre_error(0), i_error_limit(1.5) {}
    double calculate(double target, double input, double dt, double target_rate){
        double error = target - input;
        
        double i_error = error;
        if(i_error>i_error_limit) i_error = i_error_limit;
        else if(i_error<-i_error_limit) i_error = -i_error_limit;
        
        double P_term = Kp*error;
        
        integral += i_error*dt;
        double I_term = Ki * integral;
        
        double derivative = target_rate;
        double D_term = Kd * derivative;
        
        double controlOutput = P_term + I_term + D_term;
        
        pre_error = error;
        
        return controlOutput; 
    }
};

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


//--------------------------------------------------------
volatile double linear_velocity_x = 0;
volatile double linear_velocity_y = 0;
volatile double linear_velocity_z = 0;

LowPassFilter lpf_x(0.5, 0.01);  // cutoff frequency and 0.01s sampling time
LowPassFilter lpf_z(0.5, 0.01);

geometry_msgs::Vector3 filtered_vel_msg;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Apply low-pass filter to the linear velocities
    linear_velocity_x = lpf_x.filter(msg->twist.twist.linear.x);
    linear_velocity_y = msg->twist.twist.linear.y;
    linear_velocity_z = lpf_z.filter(msg->twist.twist.linear.z);

    // Set filtered velocities in the message
    filtered_vel_msg.x = linear_velocity_x;
    filtered_vel_msg.y = linear_velocity_y;
    filtered_vel_msg.z = linear_velocity_z;
}

float RC_arr[7];//0:roll, 1:pitch, 2:yaw, 3:thrust, 4:3-step switch
void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr &array){
    for(int i=0;i<7;i++){
	RC_arr[i]=array->data[i];
    }
    return;
}

dualPIDController desired_X(0.6,0,0,0.6,0,0);//0.3 0.3 roll
dualPIDController desired_Y(0.6,0,0,0.6,0,0);//1 0.7 1 pitch
PIDController desired_Z(180,15,50);

float posX, posY, posZ;
float desired_posX = 0, desired_posY = 0, desired_posZ = 0;

float altitude = 0;

float ang_limit = 0.15; //0.1
float thrust_limit = 0.1;//10
float altitude_limit = 1;

ros::Publisher pub;
ros::Publisher t265_yaw;
geometry_msgs::Vector3 pose_cmd;
std_msgs::Float32 yaw_cmd;
//ros::Publisher filtered_vel_pub;
ros::Publisher rpy_pub;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_ctrl");
    ros::NodeHandle node;

    pub = node.advertise<geometry_msgs::Vector3>("/pose_cmd", 10);
    t265_yaw = node.advertise<std_msgs::Float32>("/yaw_cmd", 10);
    //filtered_vel_pub = node.advertise<geometry_msgs::Vector3>("/filtered_linear_velocity", 10);
    rpy_pub = node.advertise<geometry_msgs::Vector3>("/rpy", 10);
    
    ros::Subscriber devo=node.subscribe("/PPM", 10, &arrayCallback);
    ros::Subscriber odom_sub = node.subscribe("/camera/odom/sample", 10, odomCallback);//linear vel

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);//position

    ros::Rate rate(100);
    while (node.ok()) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("new_origin", "drone_center", ros::Time(0));
            posX = transformStamped.transform.translation.x - 0.13;
            posY = transformStamped.transform.translation.y;
            posZ = transformStamped.transform.translation.z - 0.12;

            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w
            );
            
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            //ROS_INFO("(roll, pitch): (%f, %f)", roll, pitch);
            
            geometry_msgs::Vector3 rpy_msg;
            rpy_msg.x = roll;
            rpy_msg.y = pitch;
            rpy_msg.z = yaw;
            rpy_pub.publish(rpy_msg);
            
            yaw_cmd.data = yaw;
            yaw *=-1;            
            // 좌표 회전 변환
            float posX_rot = (posX * cos(yaw) - posY * sin(yaw));
            float posY_rot = (posX * sin(yaw) + posY * cos(yaw));
            
            if(RC_arr[6] < 1100) RC_arr[6] = 1100;
            else if (RC_arr[6] > 1900) RC_arr[6] = 1900;
            //desired_posY = (RC_arr[6]-1100)/800;
            
            altitude = RC_arr[3];
            desired_posZ = altitude_limit*(((double)1500-altitude)/(double)400)+1; //1100~1900 -1500 -400~400 /400 -1~1
            
            pose_cmd.x = desired_X.calculate(desired_posY, posY_rot, -linear_velocity_y, 0.01);      
            pose_cmd.y = desired_Y.calculate(desired_posX, posX_rot, -linear_velocity_x, 0.01);
            pose_cmd.z = desired_Z.calculate(desired_posZ, posZ, 0.01, linear_velocity_z);
		
            if (pose_cmd.x > ang_limit) pose_cmd.x = ang_limit;
            else if (pose_cmd.x < -ang_limit) pose_cmd.x = -ang_limit;
            if (pose_cmd.y > ang_limit) pose_cmd.y = ang_limit;
            else if (pose_cmd.y < -ang_limit) pose_cmd.y = -ang_limit;
            //if (pose_cmd.z > thrust_limit) pose_cmd.z = thrust_limit;
            //else if (pose_cmd.z < -thrust_limit) pose_cmd.z = -thrust_limit;
	    //ROS_INFO("pose_T_d: %f", pose_cmd.z);
            pub.publish(pose_cmd);
            t265_yaw.publish(yaw_cmd);
            //filtered_vel_pub.publish(filtered_vel_msg);

            //ROS_INFO("(x, y): (%.2f, %.2f)", posX, posY);
            //ROS_INFO("(x, y): (%.2f, %.2f)", posX_rot, posY_rot);
            //ROS_INFO("linear y: %.2f, linear x: %.2f", linear_velocity_y,linear_velocity_x);
            //ROS_INFO("roll: %.2f, pitch: %.2f", pose_cmd.x,pose_cmd.y);
            

        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

