#include "pid.h"	
#include "LIB.h"
#include "mpu6050.h"

u8 pp[]={0,0,0x0d,0x0a};

float yaw=0;                  //ת��������
float yaw_acc_error=0;            //yaw�ۻ����

#define TEN_MS_ERROR   0.00004230     //yawÿ10ms������Ư�ƵĶ���

float	Mechanical_angle=0;			    		  //��е��ֵ
float	Set_angle=0;											//�趨ƽ�⳵�Ƕ�
int		Balance_Pwm=0,Velocity_Pwm=0,Turn_Pwm=0;
int		PWMA=0,PWMB=0;
int AIN2=0, AIN1=0,BIN1=0,	BIN2=0;
/************************************************
//10ms 50����������1s 500�����������һȦ15�����������ٱ�1��36����һȦ15*36=540������������ֱ��50mm��
��һ�������ߵľ���Ϊ50*pi/540 mm����ʱ�����ٶ�Ϊ500*50*pi/540=145.444mm/s
**************************************************/
int		M1_PWM=0,M2_PWM=0;					    	 //������������ո��������PWM
int Set_Encoder_A=0;										 //�趨����ٶȣ�0-150���˵�����ת�ٵı�����ֵ��150����
int Set_Encoder_B=0;									   //�趨����ٶȣ�0-150���˵�����ת�ٵı�����ֵ��150����


/**************************************************************************
�������ܣ�PID���ƴ��붼������		
**************************************************************************/
void PID_Process(u8 Encoder_A,u8 Encoder_B)
{
//	pp[0]=Roll;
//	pp[1]=gyro[0];
//	Usart_RTX1(1,TX,pp);
	yaw_acc_error += TEN_MS_ERROR;							 			//===yawƯ������ۼ�

	Balance_Pwm  = balance_PD(Roll,gyro[0]);		 			//roll(�����)���ظ����������ٶȣ���������
	Velocity_Pwm = velocity_PI(Encoder_A,Encoder_B);
//	Turn_Pwm = turn_PD(Encoder_A,Encoder_B,gyro[2]);
//	Velocity_Pwm = (v_Pid(Encoder_A,Set_Encoder_A)+v_Pid(Encoder_B,Set_Encoder_B))/2;
	
//	M1_PWM=Velocity_Pwm;			
//	M2_PWM=Velocity_Pwm;
	M1_PWM = Balance_Pwm;			
	M2_PWM = Balance_Pwm;
	
//	M1_PWM = Balance_Pwm + Velocity_Pwm + Turn_Pwm;			
//	M2_PWM = Balance_Pwm + Velocity_Pwm - Turn_Pwm;
	
//	M1_PWM = Balance_Pwm + Velocity_Pwm;			
//	M2_PWM = Balance_Pwm + Velocity_Pwm;
	
	Xianfu_Pwm();
	Set_Pwm(M1_PWM,M2_PWM);												  //===��ֵ��PWM�Ĵ��� 
}


/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  
	  temp=-a;  
	else 
	  temp=a;
	return temp;
}


/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=9500;    //===PWM������10000 ������8000

    if(M1_PWM<-Amplitude) M1_PWM=-Amplitude;	 
		if(M1_PWM>Amplitude)  M1_PWM=Amplitude;	
	  if(M2_PWM<-Amplitude) M2_PWM=-Amplitude;	
		if(M2_PWM>Amplitude)  M2_PWM=Amplitude;		
}


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
	u16 Motor_Dead=500;                 //�������
	if(moto1>0)  { AIN2=0, AIN1=1;}
	else         { AIN2=1, AIN1=0;}
	if(moto2>0)	 { BIN1=0,	BIN2=1;}
	else         { BIN1=1,	BIN2=0;}
		Gpiox_out(A,3,BIN2);
		Gpiox_out(A,4,BIN1);
		Gpiox_out(A,6,AIN1);
		Gpiox_out(A,7,AIN2);	
		PWMA=myabs(moto1)+Motor_Dead;
		PWMB=myabs(moto2)+Motor_Dead;
		Pwm_out(A,0,PWMB);
		Pwm_out(A,1,PWMA); 


//	pp[0]=i;
//	pp[1]=j;
//	Usart_RTX1(1,TX,pp);
}


/**************************************************************************
�������ܣ�ֱ����PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
**************************************************************************/
int balance_PD(float Angle,float Gyro)
{  
	float Bias;
	float Balance_Kp=600,Balance_Kd=0.6;////900 0.9 
	int balance;
	Bias=(Angle-	Mechanical_angle-Set_angle);               //===���ƽ��ĽǶ���ֵ �ͻ�е���
	balance=Balance_Kp*Bias+Gyro*Balance_Kd;   		//===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	return balance;
}


/**************************************************************************
�������ܣ��ٶ�PI����
��ڲ�����������ֵ���趨ֵ
����  ֵ���ٶȿ���PWM
**************************************************************************/
//int v_Pid(u16 Encoder,u16 Target)
//{
//	static float Kp=60,Ki=0.3;
//	static int Bias=0,v_PWM=0,Last_bias=0;
//	
//	Bias=Target-Encoder;              //����ƫ��
//	v_PWM+=Kp*(Bias-Last_bias)+Ki*Bias; //����ʽPI������
//	Last_bias=Bias;                   //������һ��ƫ��

//	return v_PWM;
//}

/**************************************************************************
�������ܣ��ٶȻ�PI���� �޸�ǰ�������ٶ�
��ڲ��������ֱ����������ֱ�����
����  ֵ���ٶȿ���PWM
**************************************************************************/
int velocity_PI(int encoder_left,int encoder_right)
{  
	static float Velocity=0,Encoder_Least=0,Encoder=0,Movement=0;
	static float Encoder_Integral=0;

	float Velocity_Kp=50,Velocity_Ki=Velocity_Kp/200.0;//60
//	float Velocity_Kp=0,   Velocity_Ki=Velocity_Kp/200.0;               //===������
	//=============�ٶ�PI������=======================//	
	Encoder_Least =(encoder_left + encoder_right)-(Set_Encoder_A+Set_Encoder_B); //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶ�
	Encoder *= 0.8;		                                               				 //===һ�׵�ͨ�˲���       
	Encoder += Encoder_Least*0.2;	                                   				 //===һ�׵�ͨ�˲���    
	Encoder_Integral +=Encoder;                                      			   //===���ֳ�λ�� ����ʱ�䣺5ms
	Encoder_Integral=Encoder_Integral-Movement;                      			   //===����ң�������ݣ�����ǰ������
	if(Encoder_Integral>20000)  Encoder_Integral=20000;                			 //===�����޷�
	if(Encoder_Integral<-20000)	Encoder_Integral=-20000;              			 //===�����޷�	
	Velocity = Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;       			 //===�ٶȿ���	
	return Velocity;
}











