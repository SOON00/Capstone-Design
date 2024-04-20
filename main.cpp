#include "../include/pca9685/pca9685.h"
#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <unistd.h>

#include <std_msgs/Int16MultiArray.h>

int PWM[4];//PWM data

std_msgs::Int16MultiArray bldc_test;

void PWMCallback(const std_msgs::Int16MultiArray::ConstPtr &array);

PCA9685 pca9685(0x40, 1);

int mapping(double value,double min_pwm, double max_pwm,double min_pulse,double max_pulse){
    double pwm_range = max_pwm-min_pwm;//800
    double pulse_range = max_pulse-min_pulse;//410
    long double scale_factor = pwm_range/pulse_range;//800/410
    double result = min_pulse+((value-min_pwm)*pulse_range/pwm_range);//100+x*410/800
    return result;

}
void setServo(int channel, int pulse_width) {
    pca9685.setPWM(channel, 0, pulse_width);
}

int min_pulse = 205;
int max_pulse = 450;

int motor1,motor2,motor3,motor4;
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "pca9685_node");
    ros::NodeHandle nh;
    
    ros::Publisher bldc=nh.advertise<std_msgs::Int16MultiArray>("bldc_test", 100);
    ros::Subscriber cmd=nh.subscribe("/PWM", 10, &PWMCallback);
    ros::Rate loop_rate(100);
    pca9685.init();
    /*ROS_INFO("max_pulse : %d",max_pulse);
    pca9685.setPWM(0, 0, max_pulse);
    pca9685.setPWM(1, 0, max_pulse);
    pca9685.setPWM(2, 0, max_pulse);
    pca9685.setPWM(3, 0, max_pulse);
    sleep(5);
    ROS_INFO("min_pulse: %d",min_pulse);
    pca9685.setPWM(0, 0, min_pulse);
    pca9685.setPWM(1, 0, min_pulse);
    pca9685.setPWM(2, 0, min_pulse);
    pca9685.setPWM(3, 0, min_pulse);
    sleep(10);*/
    while(ros::ok()){
        motor1=mapping(PWM[0],100, 900,min_pulse,max_pulse);
        motor2=mapping(PWM[1],100, 900,min_pulse,max_pulse);
        motor3=mapping(PWM[2],100, 900,min_pulse,max_pulse);
        motor4=mapping(PWM[3],100, 900,min_pulse,max_pulse);
        
        pca9685.setPWM(0, 0, motor1);
        pca9685.setPWM(1, 0, motor2);
        pca9685.setPWM(2, 0, motor3);
        pca9685.setPWM(3, 0, motor4);
        
        /*pca9685.setPWM(0, 0, min_pulse);
        pca9685.setPWM(1, 0, min_pulse);
        pca9685.setPWM(2, 0, min_pulse);
        pca9685.setPWM(3, 0, min_pulse);*/
        bldc_test.data.resize(4);
        for(int i=0; i<4; i++){
            bldc_test.data[i] = PWM[i];
        }
    	bldc.publish(bldc_test);
    
        ROS_INFO("1:%d, 2:%d, 3:%d, 4:%d",motor1, motor2, motor3, motor4);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0; // very simple debug program, don't use in production
}

void PWMCallback(const std_msgs::Int16MultiArray::ConstPtr &array){
	for(int i=0;i<4;i++){
		PWM[i]=array->data[i];
	}
	return;
}
