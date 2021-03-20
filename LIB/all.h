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



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////��ʱ//
void delay_init(void);
void delay_ms(u16 nms);                       //��ʱ ����  ��ֵ������1800
void delay_us(u32 nus);    
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////ADC




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////NRF2401
#define NRF_READ_REG    0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define NRF_WRITE_REG   0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PLOAD     0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD     0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX        0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX        0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL     0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define NOP             0xFF  //�ղ���,����������״̬�Ĵ���	 
//SPI(NRF24L01)�Ĵ�����ַ
#define CONFIG          0x00  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                              //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define EN_AA           0x01  //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define EN_RXADDR       0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define SETUP_RETR      0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define RF_CH           0x05  //RFͨ��,bit6:0,����ͨ��Ƶ��;
#define RF_SETUP        0x06  //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define STATUS          0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
                              //bit5:���ݷ�������ж�;bit6:���������ж�;
#define MAX_TX  		0x10  //�ﵽ����ʹ����ж�
#define TX_OK   		0x20  //TX��������ж�
#define RX_OK   		0x40  //���յ������ж�

#define OBSERVE_TX      0x08  //���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define CD              0x09  //�ز����Ĵ���,bit0,�ز����;
#define RX_ADDR_P0      0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P1      0x0B  //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P2      0x0C  //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P3      0x0D  //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P4      0x0E  //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P5      0x0F  //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define TX_ADDR         0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define RX_PW_P0        0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1        0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2        0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3        0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4        0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5        0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define NRF_FIFO_STATUS 0x17  //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
                              //bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01������
#define NRF24L01_CE2   PGout(8) //24L01Ƭѡ�ź�
#define NRF24L01_CSN2  PGout(7) //SPIƬѡ�ź�	   
#define NRF24L01_IRQ2  PGin(6)  //IRQ������������

#define NRF24L01_CE1   PAout(4) //24L01Ƭѡ��
/*  zet6*/	
#define NRF24L01_CSN1  PCout(4) //SPIƬѡ�ź�	   
#define NRF24L01_IRQ1  PCin(5)  //IRQ������������

/*c8t6
#define NRF24L01_CSN1  PBout(10) //SPIƬѡ�ź�	   
#define NRF24L01_IRQ1  PBin(11)  //IRQ������������
*/

	
	
//24L01���ͽ������ݿ�ȶ���
#define TX_ADR_WIDTH    5   	//5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH    5   	//5�ֽڵĵ�ַ���
#define TX_PLOAD_WIDTH  32  	//32�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH  32  	//32�ֽڵ��û����ݿ��
	
#define	 RX 0
#define	 TX 1

void NRF24L01_Init2(void);						//��ʼ��
void NRF24L01_RX_Mode2(void);					//����Ϊ����ģʽ
void NRF24L01_TX_Mode2(void);					//����Ϊ����ģʽ
u8 NRF24L01_Write_Buf2(u8 reg, u8 *pBuf, u8 u8s);//д������
u8 NRF24L01_Read_Buf2(u8 reg, u8 *pBuf, u8 u8s);	//��������		  
u8 NRF24L01_Read_Reg2(u8 reg);					//���Ĵ���
u8 NRF24L01_Write_Reg2(u8 reg, u8 value);		//д�Ĵ���
u8 NRF24L01_Check2(void);						//���24L01�Ƿ����
u8 NRF24L01_TxPacket2(u8 *txbuf);				//����һ����������
u8 NRF24L01_RxPacket2(u8 *rxbuf);				//����һ����������								   	   
void NRF24L01_InitC8T6(void); 
void NRF24L01_Init1(void);						//��ʼ��
void NRF24L01_RX_Mode1(void);					//����Ϊ����ģʽ
void NRF24L01_TX_Mode1(void);					//����Ϊ����ģʽ
u8 NRF24L01_Write_Buf1(u8 reg, u8 *pBuf, u8 u8s);//д������
u8 NRF24L01_Read_Buf1(u8 reg, u8 *pBuf, u8 u8s);	//��������		  
u8 NRF24L01_Read_Reg1(u8 reg);					//���Ĵ���
u8 NRF24L01_Write_Reg1(u8 reg, u8 value);		//д�Ĵ���
u8 NRF24L01_Check1(void);						//���24L01�Ƿ����
u8 NRF24L01_TxPacket1(u8 *txbuf);				//����һ����������
u8 NRF24L01_RxPacket1(u8 *rxbuf);				//����һ����������
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////SPI
 				  	    													  
void SPI1_Init(void);			 //��ʼ��SPI��
void SPI1_SetSpeed(u8 SpeedSet); //����SPI�ٶ�   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI���߶�дһ���ֽ�
void SPI2_Init(void);			 //��ʼ��SPI��
void SPI2_SetSpeed(u8 SpeedSet); //����SPI�ٶ�   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI���߶�дһ���ֽ�




#define OLED_ADDRESS	0x78 //ͨ������0R����,������0x78��0x7A������ַ -- Ĭ��0x78

void I2C_Configuration(void);                //-- ����CPU��Ӳ��I2C
void I2C_WriteByte(uint8_t addr,uint8_t data); //-- ��Ĵ�����ַдһ��byte������
void WriteCmd(unsigned char I2C_Command);   //-- д����
void WriteDat(unsigned char I2C_Data);    ///-- д����
void OLED_Init(void);                      //-- OLED����ʼ��
void OLED_SetPos(unsigned char x, unsigned char y);// -- ������ʼ������
void OLED_Fill(unsigned char fill_Data);            //-- ȫ�����

void OLED_ON(void);                                  //-- ����
void OLED_OFF(void);                             // -- ˯��
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);//-- ��ʾ�ַ���(�����С��6*8��8*16����)
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N, unsigned char China[]);//-- ��ʾ����(������Ҫ��ȡģ��)
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);//����BMPλͼ��ʾ




void TIM_pwmWrite(u8 TIMX,u8 GPIOX,u16 PINX,u8 num,u16 arr,u16 psc);



#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART3_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����


 void Usart_RX(u8 USARTX,u8 *TRX_BUF ) ;





#define SYSTEM_SUPPORT_OS		0		//����ϵͳ�ļ����Ƿ�֧��UCOS
																	    
	 

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
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
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����








//����Ϊ��ຯ��
void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(u32 addr);	//���ö�ջ��ַ






#endif

