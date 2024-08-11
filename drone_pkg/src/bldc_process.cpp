#include "../include/PCA9685/PCA9685.h"
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

int min_pulse = 900;//1640
int max_pulse = 1550;//3200

int motor1,motor2,motor3,motor4;
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "pca9685_node");
    ros::NodeHandle nh;
    
    ros::Publisher bldc=nh.advertise<std_msgs::Int16MultiArray>("bldc_test", 10);
    ros::Subscriber cmd=nh.subscribe("/PWM", 10, &PWMCallback);
    ros::Rate loop_rate(200);
    pca9685.init();
    while(ros::ok()){
        motor1=mapping(PWM[0],200, 1800,min_pulse,max_pulse);
        motor2=mapping(PWM[1],200, 1800,min_pulse,max_pulse);
        motor3=mapping(PWM[2],200, 1800,min_pulse,max_pulse);
        motor4=mapping(PWM[3],200, 1800,min_pulse,max_pulse);
        
        pca9685.setPWM(15, 0, motor1);
        pca9685.setPWM(14, 0, motor2);
        pca9685.setPWM(0, 0, motor3);
        pca9685.setPWM(1, 0, motor4);

        bldc_test.data.resize(4);
        //for(int i=0; i<4; i++){
        //    bldc_test.data[i] = PWM[i];
        //}
	bldc_test.data[0] = motor1;
	bldc_test.data[1] = motor2;
	bldc_test.data[2] = motor3;
	bldc_test.data[3] = motor4;
    	
	bldc.publish(bldc_test);
    
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void PWMCallback(const std_msgs::Int16MultiArray::ConstPtr &array){
    for(int i=0;i<4;i++){
	PWM[i]=array->data[i];
    }
    return;
}
