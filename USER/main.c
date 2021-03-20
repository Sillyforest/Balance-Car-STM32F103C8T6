#include "LIB.h"
#include "pid.h"	
#include "ioi2c.h"
#include "mpu6050.h"

int t=0;
u8 msg[]={0,0x0d,0x0a};
int main(void)
{
	delay_init();								//延时初始化 
	Uart_init(1,115200);				//串口1初始化 蓝牙
	
	Gpiox_out_init(B,12);
	Gpiox_out_init(B,13);
	Gpiox_out_init(C,13);

	/**************************************************************************
	外部中断实现编码器输入
	PB3上升下降沿捕获, 1表示上升沿捕获，0表示下降沿捕获，10 表示上升下降沿捕获。 
	**************************************************************************/
	Extix_external(B,3,10);
	Extix_external(B,4,10);
	Extix_external(B,6,10);
	Extix_external(B,7,10);
	
	
	//TB6122控制端口
	Gpiox_out_init(A,3);//BIN2
	Gpiox_out_init(A,4);//BIN1
	Gpiox_out_init(A,5);//STBY
	Gpiox_out_init(A,6);//AIN1
	Gpiox_out_init(A,7);//AIN2
		
		Gpiox_out(A,3,0);
		Gpiox_out(A,4,0);
		Gpiox_out(A,5,1);
		Gpiox_out(A,6,0);
		Gpiox_out(A,7,0);
		
	////PWM输出初始化，电机频率3.6kHz，72M/(9999+1)/(1+1)=3600
	////Pwm_out_init(u8 byGpiox,u16 wPinx, u16 wArr,u16 wPsc);
	Pwm_out_init(A,0,9999,1);//PWMB
	Pwm_out_init(A,1,9999,1);//PWMA

	Pwm_out(A,0,0);
	Pwm_out(A,1,0);
	
	IIC_Init();                     //=====IIC初始化    读取MPU6050数据
	MPU6050_initialize();           //=====MPU6050初始化	
	DMP_Init();                     //=====初始化DMP 

	Usart_RTX1(1,TX,"abcde\r\n");
	
//////频率= (72000000/(99+1)/（7199+1))=100, 1/100=0.01s；
	Timex(4,99,7199);//10ms进入一次定时器中断

	
	getAngle(&yaw,&yaw_acc_error); //获取角度
	while(1)
	{
		t=!t; Gpiox_out(C,13,t);
		getAngle(&yaw,&yaw_acc_error); //获取角度
		
	}
}



