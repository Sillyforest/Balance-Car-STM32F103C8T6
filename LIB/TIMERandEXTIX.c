#include "LIB.h"
#include "pid.h"
#include "mpu6050.h"

int a=0,b=0,c=0;
u8 i=0,j=0;
int Control_Flag=1;
u8 BUF[]={0,0,0x0d,0x0a};

/*�жϺ������*/
void EXTI0_IRQHandler(void)     //// EXTI1_IRQHandler   EXTI2_IRQHandler.......
{
        
 		///////��ĺ���
//	a=!a;
// 	Gpiox_out(A,4,a);

	
	 EXTI_ClearITPendingBit(EXTI_Line0); //���LINE0�ϵ��жϱ�־λ /// (EXTI_Line1)(EXTI_Line2)...........
}

void EXTI1_IRQHandler(void)     //// EXTI1_IRQHandler   EXTI2_IRQHandler.......
{
	
			///////��ĺ���

	 EXTI_ClearITPendingBit(EXTI_Line1); //���LINE0�ϵ��жϱ�־λ /// (EXTI_Line1)(EXTI_Line2)...........
}

void EXTI2_IRQHandler(void)     //// EXTI1_IRQHandler   EXTI2_IRQHandler.......
{
	
			///////��ĺ���
   
	 EXTI_ClearITPendingBit(EXTI_Line2); //���LINE0�ϵ��жϱ�־λ /// (EXTI_Line1)(EXTI_Line2)...........
}

void EXTI3_IRQHandler(void)     //// EXTI1_IRQHandler   EXTI2_IRQHandler.......
{
        		///////��ĺ��� 
 		i++; 
		a=!a;
		Gpiox_out(B,13,a);
	 EXTI_ClearITPendingBit(EXTI_Line3); //���LINE0�ϵ��жϱ�־λ /// (EXTI_Line1)(EXTI_Line2)...........
}

void EXTI4_IRQHandler(void)     //// EXTI1_IRQHandler   EXTI2_IRQHandler.......
{
        		///////��ĺ���
  	i++; 
		a=!a;
		Gpiox_out(B,13,a);
	 EXTI_ClearITPendingBit(EXTI_Line4); //���LINE0�ϵ��жϱ�־λ /// (EXTI_Line1)(EXTI_Line2)...........
}

 
void EXTI9_5_IRQHandler(void)     //���ӵļ���
{ 
	if(EXTI_GetITStatus(EXTI_Line5)!= RESET)
	{ 
				///////��ĺ���
		
		 EXTI_ClearITPendingBit(EXTI_Line5); 
	}	
	
	if(EXTI_GetITStatus(EXTI_Line6)!= RESET)
	{ 
		j++; 
		b=!b;
		Gpiox_out(B,12,b);
		///////��ĺ���
		 EXTI_ClearITPendingBit(EXTI_Line6); 
	}	
	if(EXTI_GetITStatus(EXTI_Line7)!= RESET)
	{
		
		///////��ĺ���
		j++; 
		b=!b;
		Gpiox_out(B,12,b);
		EXTI_ClearITPendingBit(EXTI_Line7); 
	}	
	
	
 if(EXTI_GetITStatus(EXTI_Line8)!= RESET)
	{
 
		
				///////��ĺ���
		 EXTI_ClearITPendingBit(EXTI_Line8); 
	}	
	if(EXTI_GetITStatus(EXTI_Line9)!= RESET)
	{
 
		
				///////��ĺ���
		 EXTI_ClearITPendingBit(EXTI_Line9); 
	}	
}
 

void EXTI15_10_IRQHandler(void)     //���ӵļ���
{ 
	if(EXTI_GetITStatus(EXTI_Line10)!= RESET)
	{ 
				///////��ĺ���
		
		 EXTI_ClearITPendingBit(EXTI_Line10); 
	}	
	if(EXTI_GetITStatus(EXTI_Line11)!= RESET)
	{ 
		
				///////��ĺ���
		 EXTI_ClearITPendingBit(EXTI_Line11); 
	}	
	if(EXTI_GetITStatus(EXTI_Line12)!= RESET)
	{
		
				///////��ĺ���
 
		 EXTI_ClearITPendingBit(EXTI_Line12); 
	}	
	
 if(EXTI_GetITStatus(EXTI_Line13)!= RESET)
	{
 
		
				///////��ĺ���
		 EXTI_ClearITPendingBit(EXTI_Line13); 
	}	
	if(EXTI_GetITStatus(EXTI_Line14)!= RESET)
	{
 
		
				///////��ĺ���
		 EXTI_ClearITPendingBit(EXTI_Line14); 
	}	
}
 
   

     
	 				
			  
				

/*��ʱ���жϺ������*/



void TIM1_UP_IRQHandler(void)   //TIM1�ж�   ///////TIM1��TIM8 ��TIMx_UP_IRQHandler    ������ TIMx_IRQHandler
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)  //���TIM1�����жϷ������                   TIMx
		{
			
			///////��ĺ���


	   TIM_ClearITPendingBit(TIM1, TIM_IT_Update  );  //���TIMx�����жϱ�־           TIMx
		}
}


void TIM2_IRQHandler(void)   //TIM2�ж�   ///////TIM1��TIM8 ��TIMx_UP_IRQHandler    ������ TIMx_IRQHandler
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���TIM2�����жϷ������                   TIMx
		{
			
			
			
					///////��ĺ���
 	TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx�����жϱ�־           TIMx

	 }
}
void TIM3_IRQHandler(void)   //TIM3�ж�   ///////TIM1��TIM8 ��TIMx_UP_IRQHandler    ������ TIMx_IRQHandler
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������                   TIMx
		{
      
			 
					///////��ĺ���
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־           TIMx

		}
}

void TIM4_IRQHandler(void)   //TIM4�ж�   ///////TIM1��TIM8 ��TIMx_UP_IRQHandler    ������ TIMx_IRQHandler
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //���TIM4�����жϷ������                   TIMx
		{
			///////////////////////////
			
			BUF[0]=i; BUF[1]=Roll;
//			if(Roll>0) BUF[1]=Roll;
//			else BUF[0]=Roll;
			Usart_RTX1(1,TX,BUF);

		if((Roll<-50)||(Roll>50)) {Gpiox_out(A,5,0);}
		else {Gpiox_out(A,5,1);PID_Process(i,j);}
			
		i=0;j=0;
			////////////////////��ĺ���
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //���TIMx�����жϱ�־           TIMx

		}
}






