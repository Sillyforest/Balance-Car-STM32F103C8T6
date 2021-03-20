#include "LIB.h"
#include "pid.h"
#include "mpu6050.h"

int a=0,b=0,c=0;
u8 i=0,j=0;
int Control_Flag=1;
u8 BUF[]={0,0,0x0d,0x0a};

/*中断函数入口*/
void EXTI0_IRQHandler(void)     //// EXTI1_IRQHandler   EXTI2_IRQHandler.......
{
        
 		///////你的函数
//	a=!a;
// 	Gpiox_out(A,4,a);

	
	 EXTI_ClearITPendingBit(EXTI_Line0); //清除LINE0上的中断标志位 /// (EXTI_Line1)(EXTI_Line2)...........
}

void EXTI1_IRQHandler(void)     //// EXTI1_IRQHandler   EXTI2_IRQHandler.......
{
	
			///////你的函数

	 EXTI_ClearITPendingBit(EXTI_Line1); //清除LINE0上的中断标志位 /// (EXTI_Line1)(EXTI_Line2)...........
}

void EXTI2_IRQHandler(void)     //// EXTI1_IRQHandler   EXTI2_IRQHandler.......
{
	
			///////你的函数
   
	 EXTI_ClearITPendingBit(EXTI_Line2); //清除LINE0上的中断标志位 /// (EXTI_Line1)(EXTI_Line2)...........
}

void EXTI3_IRQHandler(void)     //// EXTI1_IRQHandler   EXTI2_IRQHandler.......
{
        		///////你的函数 
 		i++; 
		a=!a;
		Gpiox_out(B,13,a);
	 EXTI_ClearITPendingBit(EXTI_Line3); //清除LINE0上的中断标志位 /// (EXTI_Line1)(EXTI_Line2)...........
}

void EXTI4_IRQHandler(void)     //// EXTI1_IRQHandler   EXTI2_IRQHandler.......
{
        		///////你的函数
  	i++; 
		a=!a;
		Gpiox_out(B,13,a);
	 EXTI_ClearITPendingBit(EXTI_Line4); //清除LINE0上的中断标志位 /// (EXTI_Line1)(EXTI_Line2)...........
}

 
void EXTI9_5_IRQHandler(void)     //轮子的计数
{ 
	if(EXTI_GetITStatus(EXTI_Line5)!= RESET)
	{ 
				///////你的函数
		
		 EXTI_ClearITPendingBit(EXTI_Line5); 
	}	
	
	if(EXTI_GetITStatus(EXTI_Line6)!= RESET)
	{ 
		j++; 
		b=!b;
		Gpiox_out(B,12,b);
		///////你的函数
		 EXTI_ClearITPendingBit(EXTI_Line6); 
	}	
	if(EXTI_GetITStatus(EXTI_Line7)!= RESET)
	{
		
		///////你的函数
		j++; 
		b=!b;
		Gpiox_out(B,12,b);
		EXTI_ClearITPendingBit(EXTI_Line7); 
	}	
	
	
 if(EXTI_GetITStatus(EXTI_Line8)!= RESET)
	{
 
		
				///////你的函数
		 EXTI_ClearITPendingBit(EXTI_Line8); 
	}	
	if(EXTI_GetITStatus(EXTI_Line9)!= RESET)
	{
 
		
				///////你的函数
		 EXTI_ClearITPendingBit(EXTI_Line9); 
	}	
}
 

void EXTI15_10_IRQHandler(void)     //轮子的计数
{ 
	if(EXTI_GetITStatus(EXTI_Line10)!= RESET)
	{ 
				///////你的函数
		
		 EXTI_ClearITPendingBit(EXTI_Line10); 
	}	
	if(EXTI_GetITStatus(EXTI_Line11)!= RESET)
	{ 
		
				///////你的函数
		 EXTI_ClearITPendingBit(EXTI_Line11); 
	}	
	if(EXTI_GetITStatus(EXTI_Line12)!= RESET)
	{
		
				///////你的函数
 
		 EXTI_ClearITPendingBit(EXTI_Line12); 
	}	
	
 if(EXTI_GetITStatus(EXTI_Line13)!= RESET)
	{
 
		
				///////你的函数
		 EXTI_ClearITPendingBit(EXTI_Line13); 
	}	
	if(EXTI_GetITStatus(EXTI_Line14)!= RESET)
	{
 
		
				///////你的函数
		 EXTI_ClearITPendingBit(EXTI_Line14); 
	}	
}
 
   

     
	 				
			  
				

/*定时器中断函数入口*/



void TIM1_UP_IRQHandler(void)   //TIM1中断   ///////TIM1和TIM8 用TIMx_UP_IRQHandler    其他用 TIMx_IRQHandler
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)  //检查TIM1更新中断发生与否                   TIMx
		{
			
			///////你的函数


	   TIM_ClearITPendingBit(TIM1, TIM_IT_Update  );  //清除TIMx更新中断标志           TIMx
		}
}


void TIM2_IRQHandler(void)   //TIM2中断   ///////TIM1和TIM8 用TIMx_UP_IRQHandler    其他用 TIMx_IRQHandler
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM2更新中断发生与否                   TIMx
		{
			
			
			
					///////你的函数
 	TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx更新中断标志           TIMx

	 }
}
void TIM3_IRQHandler(void)   //TIM3中断   ///////TIM1和TIM8 用TIMx_UP_IRQHandler    其他用 TIMx_IRQHandler
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否                   TIMx
		{
      
			 
					///////你的函数
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志           TIMx

		}
}

void TIM4_IRQHandler(void)   //TIM4中断   ///////TIM1和TIM8 用TIMx_UP_IRQHandler    其他用 TIMx_IRQHandler
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //检查TIM4更新中断发生与否                   TIMx
		{
			///////////////////////////
			
			BUF[0]=i; BUF[1]=Roll;
//			if(Roll>0) BUF[1]=Roll;
//			else BUF[0]=Roll;
			Usart_RTX1(1,TX,BUF);

		if((Roll<-50)||(Roll>50)) {Gpiox_out(A,5,0);}
		else {Gpiox_out(A,5,1);PID_Process(i,j);}
			
		i=0;j=0;
			////////////////////你的函数
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIMx更新中断标志           TIMx

		}
}






