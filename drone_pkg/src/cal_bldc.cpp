#include "../include/PCA9685/PCA9685.h"
#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <unistd.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

int PPM[7];//PPM data

std_msgs::Int16MultiArray bldc_test;

void PPMCallback(const std_msgs::Float32MultiArray::ConstPtr &array);

PCA9685 pca9685(0x40, 1);

int mapping(double value,double min_pwm, double max_pwm,double min_pulse,double max_pulse){
    double pwm_range = max_pwm-min_pwm;//800
    double pulse_range = max_pulse-min_pulse;//410
    long double scale_factor = pwm_range/pulse_range;//800/410
    double result = min_pulse+((value-min_pwm)*pulse_range/pwm_range);//100+x*410/800
    return result;
}

int constrain(int x){
    if (x>=1800) x = 1800;
    else if (x<=200) x = 200;
    return x;
}

void setServo(int channel, int pulse_width) {
    pca9685.setPWM(channel, 0, pulse_width);
}

int min_pulse = 900;//1640
int max_pulse = 1550;//3200

int motor1,motor2,motor3,motor4;
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "pca9685_node");
    ros::NodeHandle nh;
    ros::Publisher bldc=nh.advertise<std_msgs::Int16MultiArray>("bldc_test", 10);
    ros::Subscriber cmd=nh.subscribe("/PPM", 10, &PPMCallback);
    ros::Rate loop_rate(200);
    pca9685.init();
    while(ros::ok()){
        motor1=2*(PPM[3]-1000);
        motor2=2*(PPM[3]-1000);
        motor3=2*(PPM[3]-1000);
        motor4=2*(PPM[3]-1000);
        motor1 = constrain(motor1);
        motor2 = constrain(motor2);
        motor3 = constrain(motor3);
        motor4 = constrain(motor4);
        motor1 = mapping(motor1,200,1800,min_pulse,max_pulse);
        motor2 = mapping(motor2,200,1800,min_pulse,max_pulse);
        motor3 = mapping(motor3,200,1800,min_pulse,max_pulse);
        motor4 = mapping(motor4,200,1800,min_pulse,max_pulse);
        
        pca9685.setPWM(15, 0, motor1);
        pca9685.setPWM(14, 0, motor2);
        pca9685.setPWM(1, 0, motor3);
        pca9685.setPWM(0, 0, motor4);
        
        bldc_test.data.resize(4);
        for(int i=0; i<4; i++){
            bldc_test.data[i] = motor1;
        }
    	bldc.publish(bldc_test);
    
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0; // very simple debug program, don't use in production
}

void PPMCallback(const std_msgs::Float32MultiArray::ConstPtr &array){
	for(int i=0;i<7;i++){
		PPM[i]=array->data[i];
	}
	return;
}
