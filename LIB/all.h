#ifndef __all_h
#define __all_h
#include "all.h"
#include "stm32f10x.h"


#define A 1
#define B 2
#define C 3
#define D 4
#define E 5
#define F 6
#define G 7

#define Pin0 GPIO_Pin_0
#define Pin1 GPIO_Pin_1
#define Pin2 GPIO_Pin_2
#define Pin3 GPIO_Pin_3
#define Pin4 GPIO_Pin_4
#define Pin5 GPIO_Pin_5
#define Pin6 GPIO_Pin_6
#define Pin7 GPIO_Pin_7
#define Pin8 GPIO_Pin_8
#define Pin9 GPIO_Pin_9
#define Pin10 GPIO_Pin_10
#define Pin11 GPIO_Pin_11
#define Pin12 GPIO_Pin_12
#define Pin13 GPIO_Pin_13
#define Pin14 GPIO_Pin_14
#define Pin15 GPIO_Pin_15




void Adc_Init(u8 ADCX ,u8 PINX );
u16  Get_Adc(u8 ch,u8 ADCX); 
u16 Get_Adc_Average(u8 ch,u8 times,u8 ADCX); 



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////延时//
void delay_init(void);
void delay_ms(u16 nms);                       //延时 毫秒  赋值不大于1800
void delay_us(u32 nus);    
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////ADC




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////NRF2401
#define NRF_READ_REG    0x00  //读配置寄存器,低5位为寄存器地址
#define NRF_WRITE_REG   0x20  //写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD     0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP             0xFF  //空操作,可以用来读状态寄存器	 
//SPI(NRF24L01)寄存器地址
#define CONFIG          0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                              //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA           0x01  //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR       0x02  //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR      0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH           0x05  //RF通道,bit6:0,工作通道频率;
#define RF_SETUP        0x06  //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS          0x07  //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
                              //bit5:数据发送完成中断;bit6:接收数据中断;
#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断

#define OBSERVE_TX      0x08  //发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define CD              0x09  //载波检测寄存器,bit0,载波检测;
#define RX_ADDR_P0      0x0A  //数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1      0x0B  //数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P2      0x0C  //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3      0x0D  //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4      0x0E  //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5      0x0F  //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define TX_ADDR         0x10  //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define RX_PW_P0        0x11  //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12  //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13  //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14  //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15  //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16  //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define NRF_FIFO_STATUS 0x17  //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
                              //bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01操作线
#define NRF24L01_CE2   PGout(8) //24L01片选信号
#define NRF24L01_CSN2  PGout(7) //SPI片选信号	   
#define NRF24L01_IRQ2  PGin(6)  //IRQ主机数据输入

#define NRF24L01_CE1   PAout(4) //24L01片选信
/*  zet6*/	
#define NRF24L01_CSN1  PCout(4) //SPI片选信号	   
#define NRF24L01_IRQ1  PCin(5)  //IRQ主机数据输入

/*c8t6
#define NRF24L01_CSN1  PBout(10) //SPI片选信号	   
#define NRF24L01_IRQ1  PBin(11)  //IRQ主机数据输入
*/

	
	
//24L01发送接收数据宽度定义
#define TX_ADR_WIDTH    5   	//5字节的地址宽度
#define RX_ADR_WIDTH    5   	//5字节的地址宽度
#define TX_PLOAD_WIDTH  32  	//32字节的用户数据宽度
#define RX_PLOAD_WIDTH  32  	//32字节的用户数据宽度
	
#define	 RX 0
#define	 TX 1

void NRF24L01_Init2(void);						//初始化
void NRF24L01_RX_Mode2(void);					//配置为接收模式
void NRF24L01_TX_Mode2(void);					//配置为发送模式
u8 NRF24L01_Write_Buf2(u8 reg, u8 *pBuf, u8 u8s);//写数据区
u8 NRF24L01_Read_Buf2(u8 reg, u8 *pBuf, u8 u8s);	//读数据区		  
u8 NRF24L01_Read_Reg2(u8 reg);					//读寄存器
u8 NRF24L01_Write_Reg2(u8 reg, u8 value);		//写寄存器
u8 NRF24L01_Check2(void);						//检查24L01是否存在
u8 NRF24L01_TxPacket2(u8 *txbuf);				//发送一个包的数据
u8 NRF24L01_RxPacket2(u8 *rxbuf);				//接收一个包的数据								   	   
void NRF24L01_InitC8T6(void); 
void NRF24L01_Init1(void);						//初始化
void NRF24L01_RX_Mode1(void);					//配置为接收模式
void NRF24L01_TX_Mode1(void);					//配置为发送模式
u8 NRF24L01_Write_Buf1(u8 reg, u8 *pBuf, u8 u8s);//写数据区
u8 NRF24L01_Read_Buf1(u8 reg, u8 *pBuf, u8 u8s);	//读数据区		  
u8 NRF24L01_Read_Reg1(u8 reg);					//读寄存器
u8 NRF24L01_Write_Reg1(u8 reg, u8 value);		//写寄存器
u8 NRF24L01_Check1(void);						//检查24L01是否存在
u8 NRF24L01_TxPacket1(u8 *txbuf);				//发送一个包的数据
u8 NRF24L01_RxPacket1(u8 *rxbuf);				//接收一个包的数据
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////SPI
 				  	    													  
void SPI1_Init(void);			 //初始化SPI口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI总线读写一个字节
void SPI2_Init(void);			 //初始化SPI口
void SPI2_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI总线读写一个字节




#define OLED_ADDRESS	0x78 //通过调整0R电阻,屏可以0x78和0x7A两个地址 -- 默认0x78

void I2C_Configuration(void);                //-- 配置CPU的硬件I2C
void I2C_WriteByte(uint8_t addr,uint8_t data); //-- 向寄存器地址写一个byte的数据
void WriteCmd(unsigned char I2C_Command);   //-- 写命令
void WriteDat(unsigned char I2C_Data);    ///-- 写数据
void OLED_Init(void);                      //-- OLED屏初始化
void OLED_SetPos(unsigned char x, unsigned char y);// -- 设置起始点坐标
void OLED_Fill(unsigned char fill_Data);            //-- 全屏填充

void OLED_ON(void);                                  //-- 唤醒
void OLED_OFF(void);                             // -- 睡眠
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);//-- 显示字符串(字体大小有6*8和8*16两种)
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N, unsigned char China[]);//-- 显示中文(中文需要先取模，)
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);//测试BMP位图显示




void TIM_pwmWrite(u8 TIMX,u8 GPIOX,u16 PINX,u8 num,u16 arr,u16 psc);



#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART3_RX 			1		//使能（1）/禁止（0）串口1接收


 void Usart_RX(u8 USARTX,u8 *TRX_BUF ) ;





#define SYSTEM_SUPPORT_OS		0		//定义系统文件夹是否支持UCOS
																	    
	 

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入








//以下为汇编函数
void WFI_SET(void);		//执行WFI指令
void INTX_DISABLE(void);//关闭所有中断
void INTX_ENABLE(void);	//开启所有中断
void MSR_MSP(u32 addr);	//设置堆栈地址






#endif

