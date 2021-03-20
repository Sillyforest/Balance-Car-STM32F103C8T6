#include "pid.h"	
#include "LIB.h"
#include "mpu6050.h"

u8 pp[]={0,0,0x0d,0x0a};

float yaw=0;                  //转向陀螺仪
float yaw_acc_error=0;            //yaw累积误差

#define TEN_MS_ERROR   0.00004230     //yaw每10ms的向上漂移的度数

float	Mechanical_angle=0;			    		  //机械中值
float	Set_angle=0;											//设定平衡车角度
int		Balance_Pwm=0,Velocity_Pwm=0,Turn_Pwm=0;
int		PWMA=0,PWMB=0;
int AIN2=0, AIN1=0,BIN1=0,	BIN2=0;
/************************************************
//10ms 50个计数，即1s 500个计数，电机一圈15个计数，减速比1：36，则一圈15*36=540个计数，轮子直径50mm，
则一个计数走的距离为50*pi/540 mm，此时车子速度为500*50*pi/540=145.444mm/s
**************************************************/
int		M1_PWM=0,M2_PWM=0;					    	 //计算出来的最终赋给电机的PWM
int Set_Encoder_A=0;										 //设定电机速度，0-150，此电机最快转速的编码器值在150左右
int Set_Encoder_B=0;									   //设定电机速度，0-150，此电机最快转速的编码器值在150左右


/**************************************************************************
函数功能：PID控制代码都在里面		
**************************************************************************/
void PID_Process(u8 Encoder_A,u8 Encoder_B)
{
//	pp[0]=Roll;
//	pp[1]=gyro[0];
//	Usart_RTX1(1,TX,pp);
	yaw_acc_error += TEN_MS_ERROR;							 			//===yaw漂移误差累加

	Balance_Pwm  = balance_PD(Roll,gyro[0]);		 			//roll(横滚角)（回复力）、角速度（阻尼力）
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
	Set_Pwm(M1_PWM,M2_PWM);												  //===赋值给PWM寄存器 
}


/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
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
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=9500;    //===PWM满幅是10000 限制在8000

    if(M1_PWM<-Amplitude) M1_PWM=-Amplitude;	 
		if(M1_PWM>Amplitude)  M1_PWM=Amplitude;	
	  if(M2_PWM<-Amplitude) M2_PWM=-Amplitude;	
		if(M2_PWM>Amplitude)  M2_PWM=Amplitude;		
}


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
	u16 Motor_Dead=500;                 //电机死区
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
函数功能：直立环PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/
int balance_PD(float Angle,float Gyro)
{  
	float Bias;
	float Balance_Kp=600,Balance_Kd=0.6;////900 0.9 
	int balance;
	Bias=(Angle-	Mechanical_angle-Set_angle);               //===求出平衡的角度中值 和机械相关
	balance=Balance_Kp*Bias+Gyro*Balance_Kd;   		//===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	return balance;
}


/**************************************************************************
函数功能：速度PI控制
入口参数：编码器值、设定值
返回  值：速度控制PWM
**************************************************************************/
//int v_Pid(u16 Encoder,u16 Target)
//{
//	static float Kp=60,Ki=0.3;
//	static int Bias=0,v_PWM=0,Last_bias=0;
//	
//	Bias=Target-Encoder;              //计算偏差
//	v_PWM+=Kp*(Bias-Last_bias)+Ki*Bias; //增量式PI控制器
//	Last_bias=Bias;                   //保存上一次偏差

//	return v_PWM;
//}

/**************************************************************************
函数功能：速度环PI控制 修改前进后退速度
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
**************************************************************************/
int velocity_PI(int encoder_left,int encoder_right)
{  
	static float Velocity=0,Encoder_Least=0,Encoder=0,Movement=0;
	static float Encoder_Integral=0;

	float Velocity_Kp=50,Velocity_Ki=Velocity_Kp/200.0;//60
//	float Velocity_Kp=0,   Velocity_Ki=Velocity_Kp/200.0;               //===调试用
	//=============速度PI控制器=======================//	
	Encoder_Least =(encoder_left + encoder_right)-(Set_Encoder_A+Set_Encoder_B); //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度
	Encoder *= 0.8;		                                               				 //===一阶低通滤波器       
	Encoder += Encoder_Least*0.2;	                                   				 //===一阶低通滤波器    
	Encoder_Integral +=Encoder;                                      			   //===积分出位移 积分时间：5ms
	Encoder_Integral=Encoder_Integral-Movement;                      			   //===接收遥控器数据，控制前进后退
	if(Encoder_Integral>20000)  Encoder_Integral=20000;                			 //===积分限幅
	if(Encoder_Integral<-20000)	Encoder_Integral=-20000;              			 //===积分限幅	
	Velocity = Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;       			 //===速度控制	
	return Velocity;
}











