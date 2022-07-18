#include "pid.h"

//输入角度和角速度，返回PWM
int Balance_Up(float angel,float gyro){
	int kp=100;
	int ki=1;
	int balance=kp*angel+ki*gyro;
	return balance;
}
int Balance_speed(int Encoder_Left,int Encoder_Right){
	static float Velocity,Encoder_Least,Encoder;

	static float Encoder_Integral;

	float kp=80,ki=0.4;

	Encoder_Least =(Encoder_Left+Encoder_Right)-0;

	Encoder *= 0.7;

	Encoder += Encoder_Least*0.3;

	Encoder_Integral +=Encoder;

	Velocity=Encoder*kp+Encoder_Integral*ki;

	return Velocity;
}
