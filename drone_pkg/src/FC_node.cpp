#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

std_msgs::Int16MultiArray PWM_cmd;
sensor_msgs::Imu imu_raw;
sensor_msgs::Imu imu_data;

geometry_msgs::Vector3 RPY_ang_vel;
geometry_msgs::Quaternion q;

const double PI = 3.1415926;

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
    angle_Kp(angle_p), angle_Ki(angle_i), angle_Kd(angle_d), rate_Kp(rate_p), rate_Ki(rate_i), rate_Kd(rate_d), prevError(0), angle_integral(0), rate_integral(0), i_limit(2),prev_rate_D_term(0) {}
    double calculate(double target, double angle_input, double rate_input, double dt){
        double angle_error = target - angle_input;
        
        double angle_P_term = angle_Kp*angle_error;
        
        angle_integral += angle_error*dt;
        if(fabs(angle_integral)>i_limit)	angle_integral=(angle_integral/fabs(angle_integral))*i_limit;
        double angle_I_term = angle_Ki * angle_integral;
        
        double angle_D_term = angle_Kd * rate_input;
        
        double target_rate = angle_P_term + angle_I_term + angle_D_term;
        
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

class PIDController{//yaw control
private:
    double Kp;
    double Ki;
    double Kd;

    double integral;
    double i_limit;
public:
    PIDController(double p, double i, double d) :
    Kp(p), Ki(i), Kd(d), integral(0), i_limit(2) {}
    double calculate(double target, double input, double rate_input, double dt){
        double error = target - input;
        
        double P_term = Kp*error;
        
        integral += error*dt;
        if(fabs(integral)>i_limit) integral=(integral/fabs(integral))*i_limit;
        double I_term = Ki * integral;
        
        double D_term = Kd * rate_input;
        
        double controlOutput = P_term + I_term + D_term;
        
        return controlOutput; 
    }
};//PID control class

//---------------------------------------------------------------
int RC_arr[7];//0:roll, 1:pitch, 2:yaw, 3:thrust, 4:3-step switch

double F1=0;//desired propeller 1 force
double F2=0;//desired propeller 2 force
double F3=0;//desired propeller 3 force
double F4=0;//desired propeller 4 force

double desired_roll = 0;//desired roll angle            목표 롤값
double desired_pitch = 0;//desired pitch angle          목표 피치값
double desired_yaw = 0;//desired yaw angle              목표 요값
double desired_yaw_vel = 0;//yaw increment tangent      각속도 목표값
double desired_thrust = 0;//desired thrust              목표 스러스트

double takeoff = 0;

//--------------------------------------------------------
static double rp_limit=0.2;//(rad)       롤피치 각도제한 
static double yaw_vel_limit=0.001;//(rad/s) 요 각속도 제한

static double T_limit=100;//(N)추력 제한
static double yaw_limit=0;

//Function declaration====================================
void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr &array);
void tau_to_PWM(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des);
double Force_to_PWM(double F, double Thrust);
void rpyT_ctrl(double roll_d, double pitch_d, double yaw_d, double Thrust_d);
void imu_Callback(const sensor_msgs::Imu::ConstPtr &imu_data);
void poseCmdCallback(const geometry_msgs::Vector3::ConstPtr& msg);
void yawCmdCallback(const std_msgs::Float32::ConstPtr& msg);
double constrain(double F);
//--------------------------------------------------------
int thrust = 0;

double integ_limit=0.5;

dualPIDController tau_Yaw(150,0,0,3,0,0);//100 3
//dualPIDController tau_Roll(2.5,0,0,3,3,0.3);// 1.75 2 2 0.3     2.5 3 3 0.3
//dualPIDController tau_Pitch(1.75,0,0,3,3,0.2);// 1.5 2 5 0.2    1.75 3 3 0.2
PIDController tau_Roll(3,0,0.5);
PIDController tau_Pitch(3,0,0.5);

//--------------------------------------------------------

double roll_angle=0;
double pitch_angle=0;
double yaw_angle=0;
double roll_vel=0;
double pitch_vel=0;
double yaw_vel=0;

float pose_r_d=0;
float pose_p_d=0;
float pose_T_d=0;

float t265_yaw=0;

int main(int argc, char **argv){
    ros::init(argc, argv, "FC_node");
    ros::NodeHandle nh;

    //Publisher
    ros::Publisher PWM=nh.advertise<std_msgs::Int16MultiArray>("PWM", 10);

    //Subscriber
    ros::Subscriber imu_data=nh.subscribe("/imu_data", 10, &imu_Callback);
    ros::Subscriber devo=nh.subscribe("/PPM", 10, &arrayCallback);
    ros::Subscriber pose_cmd = nh.subscribe("/pose_cmd", 10, poseCmdCallback);
    ros::Subscriber yaw_cmd = nh.subscribe("/yaw_cmd", 10, yawCmdCallback);

    ros::Rate loop_rate(100);

    int flag_imu=0;//monitoring imu's availability

    while(ros::ok()){
        if(RC_arr[5]>1500){ // Emergency Stop
            flag_imu=0;

            PWM_cmd.data.resize(4);
            PWM_cmd.data[0]=200;
            PWM_cmd.data[1]=200;
            PWM_cmd.data[2]=200;
            PWM_cmd.data[3]=200;
            }
		
        else{//5번 스위치 작동
	    if(flag_imu!=1){
	        desired_yaw=yaw_angle;//initial desired yaw setting
		flag_imu=1;
	    }		
	    desired_roll = -rp_limit*((RC_arr[4]-(double)1500)/(double)500);
	    desired_pitch = rp_limit*(-(RC_arr[2]-(double)1500)/(double)500);
                       
	    if(fabs(RC_arr[1]>1450 && RC_arr[1]<1550)) {yaw_limit = 1500;}
	    else {yaw_limit = RC_arr[1];}
	    
	    desired_yaw_vel = yaw_vel_limit*((yaw_limit-(double)1500)/(double)500);
	    desired_yaw -= desired_yaw_vel;
	    //ROS_INFO("r:%lf", yaw_angle);
	                 
	    thrust = RC_arr[3];
            desired_thrust = T_limit*(((double)1500-thrust)/(double)400)+100; //1100~1900 -1500 -400~400 /400 -1~1
            //ROS_INFO("r:%lf, p:%lf, y:%lf T:%lf", desired_roll, desired_pitch, y_d, desired_thrust);
            //ROS_INFO("R:%lf, P:%lf, Y:%lf", roll_angle, pitch_angle, yaw_angle);
             
            //rpyT_ctrl(desired_roll, desired_pitch, desired_yaw, desired_thrust); //only attitude
            rpyT_ctrl(desired_roll+pose_r_d, desired_pitch+pose_p_d, desired_yaw, desired_thrust); //attitude+position
            //rpyT_ctrl(pose_r_d, pose_p_d, yaw_angle, desired_thrust); //only position
            
            if(RC_arr[0]<1500){
                //rpyT_ctrl(desired_roll+pose_r_d, desired_pitch+pose_p_d, desired_yaw, desired_thrust);
                
                }
            else if(RC_arr[0]>=1500){
                if (takeoff < 90) {
                    takeoff += 0.2; 
                    //rpyT_ctrl(desired_roll+pose_r_d, desired_pitch+pose_p_d, desired_yaw, takeoff);
                }
                else if (takeoff >= 90){
                    //rpyT_ctrl(desired_roll+pose_r_d, desired_pitch+pose_p_d, desired_yaw, takeoff+pose_T_d);
                    //ROS_INFO("thrust:%lf", takeoff+pose_T_d);
                }
            }
        }
        
        if(fabs(roll_angle)>1 || fabs(pitch_angle)>1) { // Emergency Stop
            PWM_cmd.data.resize(4);
            PWM_cmd.data[0]=200;
            PWM_cmd.data[1]=200;
            PWM_cmd.data[2]=200;
            PWM_cmd.data[3]=200;
            PWM.publish(PWM_cmd);
            break;
        }

	PWM.publish(PWM_cmd);
	ros::spinOnce();
	loop_rate.sleep();	
    }
    return 0;
}

double yaw_nav_prev=0;
int yaw_nav_count=0;

void yawCmdCallback(const std_msgs::Float32::ConstPtr& msg){
    t265_yaw=msg->data;
    double temp_z=t265_yaw;
    if(fabs(temp_z-yaw_nav_prev) > PI){
	if(temp_z>=0)		yaw_nav_count-=1;
	else if(temp_z<0)	yaw_nav_count+=1;
    }		
    yaw_angle=temp_z+(double)2*PI*(double)yaw_nav_count;//yaw
    yaw_nav_prev=temp_z;
}

void poseCmdCallback(const geometry_msgs::Vector3::ConstPtr& msg){
    pose_r_d=msg->x;
    pose_p_d=msg->y;
    pose_T_d=msg->z;
}

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr &array){
    for(int i=0;i<7;i++){
	RC_arr[i]=array->data[i];
    }
    return;
}

void imu_Callback(const sensor_msgs::Imu::ConstPtr &imu_data){
    RPY_ang_vel = imu_data->angular_velocity;
    roll_vel=RPY_ang_vel.x;
    pitch_vel=RPY_ang_vel.y;
    yaw_vel=RPY_ang_vel.z;
    
    q = imu_data->orientation;
    
    roll_angle=atan2((q.y*q.z+q.w*q.x),(double)0.5-(q.x*q.x+q.y*q.y));//roll     
    double temp_y=(-(double)2*(q.x*q.z-q.w*q.y));
    //if(fabs(temp_y)>0.9999) temp_y=(temp_y/fabs(temp_y))*0.9999;
    pitch_angle=asin(temp_y);//pitch 
}

void rpyT_ctrl(double roll_d, double pitch_d, double yaw_d, double Thrust_d){	
    double tau_Roll_input = tau_Roll.calculate(roll_d, -roll_angle, -roll_vel, 0.005);
    double tau_Pitch_input = tau_Pitch.calculate(pitch_d, pitch_angle, pitch_vel, 0.005);
    double tau_Yaw_input = tau_Yaw.calculate(-yaw_d, -yaw_angle, -yaw_vel, 0.005);//yaw_d, yaw_angle, yaw_vel,0.005
    //ROS_INFO("Roll :%lf, yaw_vel :%lf, Yaw:%lf, yaw_angle:%lf", tau_Roll_input, yaw_vel, tau_Yaw_input, yaw_angle);
    tau_to_PWM(tau_Roll_input, tau_Pitch_input, tau_Yaw_input, Thrust_d);
}

void tau_to_PWM(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des){	
    F1 = -(1.1121 * tau_r_des + 1.1121 * tau_p_des - 0.25 * tau_y_des - 0.25 * Thrust_des);
    F2 = -(1.1121 * tau_r_des - 1.1121 * tau_p_des + 0.25 * tau_y_des - 0.25 * Thrust_des);
    F3 = -(-1.1121 * tau_r_des - 1.1121 * tau_p_des - 0.25 * tau_y_des - 0.25 * Thrust_des);
    F4 = -(-1.1121 * tau_r_des + 1.1121 * tau_p_des + 0.25 * tau_y_des - 0.25 * Thrust_des);
    //ROS_INFO("F1:%lf, F2:%lf, F3:%lf, F4:%lf", F1, F2, F3, F4);
    F1= constrain(F1);
    F2= constrain(F2);
    F3= constrain(F3);
    F4= constrain(F4);
    //ROS_INFO("F1:%lf, F2:%lf, F3:%lf, F4:%lf", F1, F2, F3, F4);
    PWM_cmd.data.resize(4);
    PWM_cmd.data[0]=Force_to_PWM(F1,Thrust_des);
    PWM_cmd.data[1]=Force_to_PWM(F2,Thrust_des);
    PWM_cmd.data[2]=Force_to_PWM(F3,Thrust_des);
    PWM_cmd.data[3]=Force_to_PWM(F4,Thrust_des);
}

double constrain(double F){
    if(F<=0) return 0;
    else if(F>1600) return 1600;
    else return F;
}

double Force_to_PWM(double F, double Thrust){
    double pwm=2*(sqrt((32*F+243.1144)/0.0012)-343.9167);
    if(pwm<=200) {pwm=200;}
    if(pwm>=1800) {pwm=1800;}
    return pwm;
}

void TakeOff(){

}
