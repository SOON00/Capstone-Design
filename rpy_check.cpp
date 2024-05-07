#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

sensor_msgs::Imu imu_data;

geometry_msgs::Vector3 RPY_ang_vel;
geometry_msgs::Quaternion q;

double roll_angle=0;
double pitch_angle=0;
double yaw_angle=0;
double roll_vel=0;
double pitch_vel=0;
double yaw_vel=0;
double yaw_nav_prev=0;
int yaw_nav_count=0;

//Function declaration====================================
void imu_Callback(const sensor_msgs::Imu::ConstPtr &imu_data);

//Main====================================================

int main(int argc, char **argv){
	ros::init(argc, argv, "RPY_check");
	ros::NodeHandle nh;

	//Subscriber
	ros::Subscriber imu_data=nh.subscribe("/imu_data", 100, &imu_Callback);
	
	ros::Rate loop_rate(300);

	int flag_imu=0;//monitoring imu's availability

	while(ros::ok()){
		ROS_INFO("R:%lf, P:%lf, Y:%lf", roll_angle, pitch_angle, yaw_angle);
		ros::spinOnce();
		loop_rate.sleep();	

	}
	return 0;
}
//--------------------------------------------------------

//Functions===============================================

void imu_Callback(const sensor_msgs::Imu::ConstPtr &imu_data){
    RPY_ang_vel = imu_data->angular_velocity;
    roll_vel=RPY_ang_vel.x;
    pitch_vel=RPY_ang_vel.y;
    yaw_vel=RPY_ang_vel.z;
    q = imu_data->orientation;
    
    //roll
    roll_angle=atan2((q.y*q.z+q.w*q.x),(double)0.5-(q.x*q.x+q.y*q.y));//roll
    
    //pitch        
    double temp_y=(-(double)2*(q.x*q.z-q.w*q.y));
    //if(fabs(temp_y)>0.9999) temp_y=(temp_y/fabs(temp_y))*0.9999;
    pitch_angle=asin(temp_y);//pitch 
	
    //yaw
    double temp_z=atan2((q.x*q.y+q.w*q.z),(double)0.5-(q.y*q.y+q.z*q.z));
    if(fabs(temp_z-yaw_nav_prev)>3.141592){
	if(temp_z>=0)		yaw_nav_count-=1;
	else if(temp_z<0)	yaw_nav_count+=1;
    }		
    yaw_angle=temp_z+(double)2*3.141592*(double)yaw_nav_count;//yaw
    yaw_nav_prev=temp_z;
}

