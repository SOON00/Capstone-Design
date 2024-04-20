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

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "pca9685_node");
    ros::NodeHandle nh;
    
    ros::Publisher bldc=nh.advertise<std_msgs::Int16MultiArray>("bldc_test", 100);
    ros::Subscriber cmd=nh.subscribe("/PWM", 3, &PWMCallback);
    //ros::Rate loop_rate(200);
    pca9685.init();
    pca9685.setPWM(0, 0, 1900);
    pca9685.setPWM(1, 0, 1900);
    pca9685.setPWM(2, 0, 1900);
    pca9685.setPWM(3, 0, 1900);
    sleep(5);
    
    pca9685.setPWM(0, 0, 1200);
    pca9685.setPWM(1, 0, 1200);
    pca9685.setPWM(2, 0, 1200);
    pca9685.setPWM(3, 0, 1200);
    sleep(5);
    while(ros::ok()){
        pca9685.setPWM(0, 0, PWM[1]+1100);
        pca9685.setPWM(1, 0, PWM[1]+1100);
        pca9685.setPWM(2, 0, PWM[1]+1100);
        pca9685.setPWM(3, 0, PWM[1]+1100);
        bldc_test.data.resize(4);
        for(int i=0; i<4; i++){
            bldc_test.data[i] = PWM[i];
        }
        //std::cout << "hello" << std::endl;
    	bldc.publish(bldc_test);
    
        ROS_INFO("1:%d, 2:%d, 3:%d, 4:%d",PWM[0], PWM[1], PWM[2], PWM[3]);
        ros::spinOnce();
        //loop_rate.sleep();
    }
    return 0; // very simple debug program, don't use in production
}

void PWMCallback(const std_msgs::Int16MultiArray::ConstPtr &array){
	for(int i=0;i<4;i++){
		PWM[i]=array->data[i];
	}
	return;
}
