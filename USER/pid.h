#ifndef __PID_H__
#define __PID_H__
#include "LIB.h"
#include "sys.h"

extern float yaw, yaw_acc_error;      //yaw累积误差
//extern float pitch,roll,yaw; 		//欧拉角
//extern short aacx,aacy,aacz;			//加速度传感器原始数据
//extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据
//extern short temp;								//温度

void PID_Process(u8 Encoder_A,u8 Encoder_B);
void Xianfu_Pwm(void);
void Set_Pwm(int moto1,int moto2);
int myabs(int a);
int v_Pid(u16 Encoder,u16 Target);
int velocity_PI(int encoder_left,int encoder_right);
int balance_PD(float Angle,float Gyro);
int turn_PD(int encoder_left,int encoder_right,float gyro);

#endif //__PID_H__
