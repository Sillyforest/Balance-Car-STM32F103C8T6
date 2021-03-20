#ifndef __PID_H__
#define __PID_H__
#include "LIB.h"
#include "sys.h"

extern float yaw, yaw_acc_error;      //yaw�ۻ����
//extern float pitch,roll,yaw; 		//ŷ����
//extern short aacx,aacy,aacz;			//���ٶȴ�����ԭʼ����
//extern short gyrox,gyroy,gyroz;	//������ԭʼ����
//extern short temp;								//�¶�

void PID_Process(u8 Encoder_A,u8 Encoder_B);
void Xianfu_Pwm(void);
void Set_Pwm(int moto1,int moto2);
int myabs(int a);
int v_Pid(u16 Encoder,u16 Target);
int velocity_PI(int encoder_left,int encoder_right);
int balance_PD(float Angle,float Gyro);
int turn_PD(int encoder_left,int encoder_right,float gyro);

#endif //__PID_H__
