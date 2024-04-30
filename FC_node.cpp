#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

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
public:
    dualPIDController(double angle_p, double angle_i, double angle_d, double rate_p, double rate_i, double rate_d) :
    angle_Kp(angle_p), angle_Ki(angle_i), angle_Kd(angle_d), rate_Kp(rate_p), rate_Ki(rate_i), rate_Kd(rate_d), prevError(0), angle_integral(0), rate_integral(0) {}
    double calculate(double target, double angle_input, double rate_input, double dt){
        double angle_error = target - angle_input;
        
        double angle_P_term = angle_Kp*angle_error;
        double target_rate = angle_P_term;
        
        double rate_error = target_rate - rate_input;
        
        double rate_P_term = rate_Kp*rate_error;
        
        rate_integral += rate_error*dt;
        double rate_I_term = rate_Ki * rate_integral;
        
        double rate_D_term = rate_Kd * (rate_error-prevError)/dt;
        
        angle_integral += angle_error*dt;
        double angle_I_term = angle_Ki * angle_integral;
        
        double angle_D_term = angle_Kd * rate_input;
        
        prevError = rate_error;
        
        
        double controlOutput = rate_P_term + rate_I_term + rate_D_term + angle_I_term + angle_D_term;
        
        return controlOutput; 
    }
};//dualPID control class

double freq=200;//controller loop frequency 0.005s
std_msgs::Int16MultiArray PWM_cmd;

int arr[7];//0:roll, 1:pitch, 2:yaw, 3:thrust, 4:3-step switch 
float imu_array[6];

//Commands================================================

//Thruster_cmd
double F1=0;//desired propeller 1 force
double F2=0;//desired propeller 2 force
double F3=0;//desired propeller 3 force
double F4=0;//desired propeller 4 force

double e_r_i=0;//roll error integration
double e_p_i=0;//pitch error integration

double tau_r_d=0;//roll  desired torque (N.m)         여기에 PID제어 들어감
double tau_p_d=0;//pitch desired torque(N.m)          여기도 마찬가지

//ud_cmd
double r_d=0;//desired roll angle             목표 롤값
double p_d=0;//desired pitch angle            목표 피치값
double y_d=0;//desired yaw angle              목표 요값
double y_d_tangent=0;//yaw increment tangent  각속도 목표값
double T_d=0;//desired thrust                 목표 스러스트

//--------------------------------------------------------

//General parameters======================================

static double rp_limit=10;//(degree)       롤피치 각도제한 
static double y_vel_limit=0.1;//(degree/s) 요 각속도 제한
static double y_d_tangent_deadzone=(double)0.05*y_vel_limit;//(rad/s)
//작은 오차에 의한 드론의 움직임 방지
static double T_limit=100;//(N)           추력 제한
static double yaw_limit=0;
//--------------------------------------------------------


//Function declaration====================================
void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr &array);
void imu_arrayCallback(const std_msgs::Float32MultiArray::ConstPtr &array);

void ud_to_PWM(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des);
double Force_to_PWM(double F, double Thrust);
void rpyT_ctrl(double roll_d, double pitch_d, double yaw_d, double Thrust_d);

double constrain(double F);
//--------------------------------------------------------


//Control gains===========================================

//integratior(PID) limitation
double integ_limit=2;

//Roll, Pitch PID gains
double Pr=2;//roll
double Pp=3;//pitch
double Ia=0;
double Da=0;
double Dp=0;

double Par=0.01;//angular roll
double Pap=0.015;//angular pitch

//Yaw PID gains
double Py=1;//1 : good
double Dy=0;

dualPIDController tau_Roll(2,0,0,1,0,0);
dualPIDController tau_Pitch(2,0,0,1,0,0);
//--------------------------------------------------------

//Main====================================================

int main(int argc, char **argv){
	ros::init(argc, argv, "FC_node");
	ros::NodeHandle nh;

	//Publisher
	ros::Publisher PWM=nh.advertise<std_msgs::Int16MultiArray>("PWM", 100);

	//Subscriber
	ros::Subscriber imu=nh.subscribe("/imu", 100, &imu_arrayCallback);
	ros::Subscriber devo=nh.subscribe("/PPM", 100, &arrayCallback);

	ros::Rate loop_rate(200);

	int flag_imu=0;//monitoring imu's availability

	while(ros::ok()){
		if(arr[5]>1500){ // Emergency Stop
		 	flag_imu=0;

		 	PWM_cmd.data.resize(4);
		 	PWM_cmd.data[0]=200;//각 모터에 대한 PWM값
		 	PWM_cmd.data[1]=200;
		 	PWM_cmd.data[2]=200;
		 	PWM_cmd.data[3]=200;
		}
		
		 else{//5번 스위치 작동
			//Initialize desired yaw
			if(flag_imu!=1){
				y_d=imu_array[2];//initial desired yaw setting
				e_r_i=0;//initialize roll integrator     I제어 초기화
				e_p_i=0;//initialize pitch integrator    I제어 초기화
	
				//if(FP_lin_acc.z<(double)-9)	flag_imu=1;
				flag_imu=1;
                //드론의 수직 Z축 방향 선형 가속도가 거의 0일 때 imu 작동하게
			}
			//TGP ctrl
			r_d=rp_limit*(-(arr[4]-(double)1500)/(double)500);
            //목표 롤 각도 최대값 곱하기 -1or1 (수신기 신호를 -1~1로 맵핑하는 느낌)
			p_d=rp_limit*(-(arr[2]-(double)1500)/(double)500);
            //목표 피치
			if(fabs(arr[1]>1450 && arr[1]<1550)) {yaw_limit = 1500;}
			else {yaw_limit = arr[1];}
			y_d_tangent=y_vel_limit*((yaw_limit-(double)1500)/(double)500);
            //목표 요 각속도
			//if(fabs(y_d_tangent)<y_d_tangent_deadzone || fabs(y_d_tangent)>y_vel_limit) y_d_tangent=0;
            //데드존안에 있지 않거나 제한값을 초과할 때 요 각속도 0으로 하여 안정화
			y_d+=y_d_tangent;
			if(y_d>180) y_d-=360;
			else if (y_d<-180) y_d+=360; 
            //요 각속도를 더하여 목표 요 각도를 만들기
			T_d=T_limit*(((double)1500-arr[3])/(double)400)+100; //1100~1900 -1500 -400~400 /400 -1~1
            //목표 추력

			ROS_INFO("r:%lf, p:%lf, y:%lf T:%lf", r_d, p_d, y_d, T_d);
			//if(yaw_limit==1500) y_d=0;//if angular_y_vel_desire is 0, y_d=0
			rpyT_ctrl(r_d, p_d, y_d, T_d);
            //목표 각도와 추력을 이용해 PWM 계산하는 함수	
			
		}
			if(fabs(imu_array[0])>35 || fabs(imu_array[1])>35) { // Emergency Stop
		 	PWM_cmd.data.resize(4);
		 	PWM_cmd.data[0]=200;//각 모터에 대한 PWM값
		 	PWM_cmd.data[1]=200;
		 	PWM_cmd.data[2]=200;
		 	PWM_cmd.data[3]=200;
		 	PWM.publish(PWM_cmd);
		 	break;             //Emergency break code
		       }
			/*if(arr[4] <= 100) { // Emergency Stop
		 	PWM_cmd.data.resize(4);
		 	PWM_cmd.data[0]=100;//각 모터에 대한 PWM값
		 	PWM_cmd.data[1]=100;
		 	PWM_cmd.data[2]=100;
		 	PWM_cmd.data[3]=100;
		 	PWM.publish(PWM_cmd);

		}*/

		PWM.publish(PWM_cmd);
		ros::spinOnce();
		loop_rate.sleep();	

	}
	return 0;
}
//--------------------------------------------------------

//Functions===============================================

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr &array){
	for(int i=0;i<7;i++){
		arr[i]=array->data[i];
	}
	return;
}

void imu_arrayCallback(const std_msgs::Float32MultiArray::ConstPtr &array){
	for(int i=0;i<6;i++){
		imu_array[i]=array->data[i];
	}
	return;
}

void rpyT_ctrl(double roll_d, double pitch_d, double yaw_d, double Thrust_d){
//원하는 rpy,thrust값 받아서 PID제어. 롤,피치의 명령 토크 계산. 
//롤,피치 명령 토크와 요 각도 편차를 이용하여 요 명령 토크 계산
//계산된 롤피치요 명령 토크, 스러스트 값을 이용하여 PWM 변환
	double e_r=roll_d-(imu_array[0]);//offset settings----------------
	double e_p=pitch_d-(imu_array[1]-4);
	double e_y=yaw_d-imu_array[2];
	double angular_velocity_roll=imu_array[3];
	double angular_velocity_pitch=imu_array[4];
	
	double rate_error = Pr*e_r-angular_velocity_roll;
	
	if(e_y>180) e_y-=360;
	else if (e_y<-180) e_y+=360;
	
	//e_r=roll_d=0;
	//e_p=pitch_d=0;
	//e_y=yaw_d=0;

	e_r_i+=e_r*((double)1/freq);//롤 에러 적분 업데이트
	if(fabs(e_r_i)>integ_limit)	e_r_i=(e_r_i/fabs(e_r_i))*integ_limit;
    //적분제어 상한선 걸어놓음
	e_p_i+=e_p*((double)1/freq);//피치 에러
	if(fabs(e_p_i)>integ_limit)	e_p_i=(e_p_i/fabs(e_p_i))*integ_limit;

	tau_r_d=0;//롤 방향 목표 토크
	tau_p_d=0;//피치 방향 목표 토크

		//PID 제어
	//tau_r_d=Pr*e_r+Ia*e_r_i+Da*(imu_array[3]);
	tau_r_d=Par*(Pr*e_r-angular_velocity_roll)+Ia*e_r_i+Da*(imu_array[3]);
	
        	//롤 각도에 대한 목표 토크 PID로 구하기
        	
	//tau_p_d=Pp*e_p+Ia*e_p_i+Dp*(-imu_array[4]);
	tau_p_d=Pap*(Pp*e_p+angular_velocity_pitch)+Ia*e_p_i+Dp*(-imu_array[4]);

	double tau_y_d=Py*e_y+Dy*(-imu_array[5]);
	
	double tau_Roll_input = tau_Roll.calculate(0, imu_array[0]*PI/180, imu_array[3]*PI/180,0.005);
	double tau_Pitch_input = tau_Pitch.calculate(0, imu_array[1]*PI/180, -imu_array[4]*PI/180,0.005);

	//ROS_INFO("Roll :%lf, Pitch :%lf, ty:%lf, Thrust_d:%lf", tau_r_d, tau_p_d, tau_y_d, Thrust_d);
	ROS_INFO("roll :%lf, pitch :%lf", tau_Roll_input, tau_Pitch_input);
	//ud_to_PWM(tau_r_d, tau_p_d, tau_y_d, Thrust_d);
	ud_to_PWM(tau_Roll_input, tau_Pitch_input, tau_y_d, Thrust_d);//tau_r_d,tau_p_d
}

void ud_to_PWM(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des){	
	F1 = -(1.5625 * tau_r_des + 1.5625 * tau_p_des - 0.25 * tau_y_des - 0.25 * Thrust_des);
	F2 = -(1.5625 * tau_r_des - 1.5625 * tau_p_des + 0.25 * tau_y_des - 0.25 * Thrust_des);
	F3 = -(-1.5625 * tau_r_des - 1.5625 * tau_p_des - 0.25 * tau_y_des - 0.25 * Thrust_des);
	F4 = -(-1.5625 * tau_r_des + 1.5625 * tau_p_des + 0.25 * tau_y_des - 0.25 * Thrust_des);
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
	//ROS_INFO("1:%d, 2:%d, 3:%d, 4:%d",PWM_cmd.data[0], PWM_cmd.data[1], PWM_cmd.data[2], PWM_cmd.data[3]);
}

double constrain(double F){
    if(F<=0) return 0;
    else if(F>1600) return 1600;
    else return F;
}

double Force_to_PWM(double F, double Thrust){

	double pwm=2*(sqrt((32*F+243.1144)/0.0012)-343.9167);
	if(pwm<=200)	{pwm=200;}
	if(pwm>=1800)	{pwm=1800;}

	return pwm;
}
