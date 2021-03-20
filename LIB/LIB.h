/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIB_h
#define __LIB_h
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
                       
#include "stdbool.h"
#include "all.h"
#include "stdlib.h"




/***************************************/

/****************************************** 
高精度定时模块

***************************************/

void Timex_time(u8 byTimex , u16 psc);             //定时模块初始化   byTimex  ：1   ，psc：分频       ,每一个CNT时间=72M/psc
void Time_chushi(u16 CNTC);                         // CNT重新装填值，一般为0；
void Time_wait(u16 CNTO);                           //等待时间到       例：Timex_time(1 , 7199); Time_chushi(0);    Time_wait(9999);       定时1,秒     
/****************************************/



/***************************************
函数名：Gpiox_in
功能：GPIO输入检测 
参数说明：byGpiox 可选 A-G，wPinx 可选 Pin0-Pin15，模式byMode为 0或1   下拉输入或上拉输入     如先定义 Gpiox_in_init(B,15,1);    后 if(  Gpiox_in(B,15 )==0)    
返回值： 返回1或0；表示有输入或无输入
时间：2020-1-29
代码作者：xi-lin
************************************/
bool Gpiox_in(u8 byGpiox,u16 wPinx );
void Gpiox_in_init(u8 byGpiox,u16 wPinx,u8 byMode); //IO初始化




/***************************************
函数名：Gpiox_out
功能：GPIO输出
参数说明：//GPIO输出byGpiox 可选 A-G，wPinx 可选 Pin0-Pin15，模式byMode为 0或1   低电平或高电平 先  定义 Gpiox_out_init(B,15);     如 GPIOx_out(B,15,1 )      GPIOB_Pin_15 高电平
返回值： 无
时间：2017-8-6
代码作者：xi-lin
************************************/

void Gpiox_out(u8 byGpiox,u16 wPinx,u8 byMode);  
void Gpiox_out_init(u8 byGpiox,u16 wPinx);

/***************************************
函数名： Delay_ms
功能：延时毫秒  
参数说明： 输入延时   毫秒  赋值不大于1800
返回值： 无
时间：2017-8-6
代码作者：xi-lin
************************************/
void Delay_ms(u32 wMs);   



/***************************************
函数名： Delay_us
功能： //延时 微妙   
参数说明： 输入延时  微秒
返回值： 无
时间：2017-8-6
代码作者：xi-lin
************************************/
void Delay_us(u32 uUs);   

// /*******************************************************************************
//  *功能描述：无线数据传输
//  *外设资源：I/0，SPI
//  *硬件环境：STM32F103ZET6
//  *支持资源：调用的其他外部文件和硬件资源等
//  *备    注：
//  *函数名： Nrf24l01_rt
//  *参数说明：NRF2.4G  spi接口   bySpix 可选 1或2。 byMode 可选RX或TX  ，p_byBuf 为接收或发生数组指针
//  *         如 Nrf24l01_rt(1,RX,Buf); spi1接口 ，接收模式 接收到的东西放在Buf[]这个数组里
//  *         接收临界延时 300us      Buf[] 数组自己定义，大小 最大Buf[32];
//  *返回值： 无
//  *****************************************************************************/


//void Nrf24l01_rt(u8 byspix,u8 bymode,u8 *p_bytmp_buf);  

/***************************************
函数名：Extix_external
功能： 外部中断
参数说明：//外部中断使能 。变量对应  byGpiox  可选  A-G 。 byLinx 可选 0-15 模式 byMode可选  1上升捕获， 0 下降捕获 ，10 上升下降捕获
         //例如 Extix_external(B,12,1）;   GPIOB_Pin_12  为外部中断口。模式选择为 1上升捕获，
返回值： 无
时间：2017-8-6
代码作者：xi-lin
************************************/
void Extix_external(u8 byGpiox,u8 byLinx,u8 byMode);




/***************************************
函数名： Timex
功能： 定时器中断 
参数说明：//定时器中断线。TIM1-TIM8，arr为计数最大值，psc为分频   默认向上计数，
         //如果arr=899 psc=0  则不分频。PWM频率=72000000/900=80Khz
         // 如果arr=200 psc=7199  则分频。PWM频率=72000000/7200 /200=50HZ
         //写法如   Timex(1,4999,7199)  为，定时器1，计数到4999,7199分频 
返回值： 无
时间：2017-8-6
代码作者：xi-lin
************************************/
void Timex(u8 byTimex ,u16 arr,u16 psc);   
                                             
																						 
																						 
/*****************************************************************************
函数名：Adc_in
功能： ADC输入 //12位
参数说明：//ADC可选 1或2 ， byCh取值  通道0-通道7 对应 GPIOA Pin0-Pin7   byTimes是byTimes次的平均值 
         //ADC 输入 返回 0-4096;  全独立   写法例如  L=Adc_in（1,0,10）  ADC1，通道0,十次平均值。
         //返回得到的ADC值给L；
返回值：收到的ADC值   0-4096
时间：2017-8-6
代码作者：xi-lin
***************************************/     
void Adc_Init(u8 ADCX ,u8 PINX );
u16 Adc_in(u8 byAdcx,u8 byCh,u8 byTimes);   



/***************************************
函数名：Oled_Showcn
功能： OLED汉字字符显示  ***** //IIC接口 PB6 SCL 和PB7 	SDL		
参数说明：汉字字体输入 x y 为坐标 ， wChina[]为字符编码数组 ，  输入方式如 	Oled_Showcn (0,0,1, wChina); 1为，汉字个数  ch为自己定义的编码数组指针
返回值：无
时间：2017-8-6
代码作者：xi-lin
************************************/   
void 	Oled_Showcn (u8 byX, u8 byY,u8 byN,u8 wChina[] );



/***************************************
函数名：Oled_Showstr
功能： OLED字符显示  ***** //IIC接口 PB6 SCL 和PB7 	SDL		
参数说明： //   英文字体输入 byx byy 为坐标 同上，bych[]为字符数组 ， byTextsize为字体大小 1为  小字体 
          //2为大字体，    输入方式如 Oled_Showstr (0,0,ch,1);  ch为自己定义的数组指针 ，或者  
          // OLED_ShowStrL (0,0,"kkkk",1);   kkkk为显示内容                       
返回值：无
时间：2017-8-6
代码作者：xi-lin
************************************/ 




void 	Oled_Showstr (u8 byX, u8 byY,u8 buCh[],u8 byTextsize); 

   /***************************************
函数名：Oled_DrawBMP
功能： OLED图片显示  ***** //IIC接口 PB6 SCL 和PB7 	SDL		
参数说明：  x0 y0 起始坐标 ，，x1，y1终止坐标。，bmp图片数组            
返回值：无
时间：2017-8-6
代码作者：xi-lin
************************************/ 
    
void Oled_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);

/***************************************
函数名：Oled_Cls
功能： 清屏
参数说明： 无     
返回值：无
时间：2017-8-6
代码作者：xi-lin
************************************/  
void Oled_Cls(void);          


/**
  *函数说明：PWM输出初始化,只适用于TIM1 TIM2 TIM3 TIM4 TIM8
  *输入参数说明：
	             ////使用  TIM1： GPIOX PINX 填  （A,8/9/10/11）     如果arr=899 psc=0  则不分频。PWM频率=72000000/900=80Khz - 
               ////使用  TIM2： GPIOX PINX 填  （A,0/1/2/3）     截止值可以写入0-899    如果PW=449   则 PWM输出 为1/2占空比
               ////使用  TIM3： GPIOX PINX 填  （B,0/1/4/5）     如果arr=200 psc=7199  则分频。PWM频率=72000000/7200 /200=50HZ
               ////使用  TIM4： GPIOX PINX 填  （B,6/7/8/9）            
               ////使用  TIM8： GPIOX PINX 填  （C,6/7/8/9）     例子Pwm_out_init(B,0,200,7199）  Pwm_out(B,0,5）  
	   +///STM32F103C8T6只有   TIM1-TIM4
	
  *返回类型：void
  */

void Pwm_out_init(u8 byGpiox,u16 wPinx, u16 wArr,u16 wPsc);
void Pwm_out(u8 byGpiox,u16 wPinx,u16 wpw);



/***************************************
函数名：Usart_RTX
功能：串口通信模式1
参数说明： 
        函数说明：串口全双工数据传输   需要0x0d 0x0a 结束符；
        byUsartx 为选择串口；如  输入参数   1  或  2  或  3     代表开启串口1或2  或3    
				byMode   输入模式  输入参数  TX 或RX   发射模式或接受模式     可互相切换
				uBottod   波特率   如   9600   115200  
				*p_byTRX_BUF    接受或发射数组   接受模式下  ，执行此函数后，数据将储存于此数组。发射模式下，发送此数组的数据，，TX模式 末尾自动添加 0x0d  0x0a    
			 
				例如填入  Usart_RTX(1,TX,115200,buff);        意思是打开串口1  。开启发送模式，波特率设为115200  ，发送数组头指针  buff
				
				
				******  
				
返回值：无
时间：2017-8-6
代码作者：xi-lin
************************************/ 
 
 
 void Uart_init(u8 byUsartx,u32 uBottod);
void  Usart_RTX1(u8 byUsartx,u8 byMode,u8 *p_byTRX_BUF ) ;

 
 

/***************************************
函数名：Usart_TRX2
功能：串口通信模式2
参数说明： 
        函数说明：串口全双工数据传输   需要0x0d 0x0a 结束符；
        byUsartx 为选择串口；如  输入参数   1  或  2  或  3     代表开启串口1或2  或3    
				byMode   输入模式  输入参数  TX 或RX   发射模式或接受模式     可互相切换
				uBottod   波特率   如   9600   115200  
				*p_byTRX_BUF    接受或发射数组   接受模式下  ，执行此函数后，数据将储存于此数组。发射模式下，发送此数组的数据，，TX模式 末尾自动添加 0x0d  0x0a    
				u8 by_bit  。发送的数据为数    最大200;
				 u8 qishi   发送的和接受的起始位标志符；
				例如填入  Usart_RTX(1,RX,115200,buff,8,0xff);        意思是打开串口1  。开启接受模式，波特率设为115200  ，发送数组头指针  buff     一共接受8个字节   接受开始标志是收到0xff   ***发送的时候0xf标志符无实际意义
				
				
				******  
				
返回值：无
时间：2018-5-4
代码作者：xi-lin
************************************/ 
void  Usart_TRX2(u8 byUsartx ,u8 byMode,u8 *p_byTRX_BUF,u8 by_bit,u8 qishi) ;   //固定位模式



//版本查询
void BXlin666(void);



#endif /*__LIB_H */
/************** (C) COPYRIGHT   *******END OF FILE****/
/****************************2018年5-4日更新：xi-lin*******************************/

