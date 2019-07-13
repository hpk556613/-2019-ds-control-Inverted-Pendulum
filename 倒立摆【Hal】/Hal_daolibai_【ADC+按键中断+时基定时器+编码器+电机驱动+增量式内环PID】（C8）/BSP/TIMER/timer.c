#include "timer.h"


/**
  * 函数功能: 非阻塞模式下定时器的回调函数
  * 输入参数: htim：定时器句柄
  * 返 回 值: 无
  * 说    明: 无
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim1))
	{
		Angle_Balance=Get_Adc_Average(3,15);               //===更新姿态			
		
		if(Position_Key==1)																//按下USER按键才开始调整
		{
			if(Times_up==1)
			{
				Times_up=0;

				/* 【外环】角度部分*/
								
//				Error_Target=Angle_Balance-Motor_AnglePID.SetPoint;
//				Para1=Inc_PID_Calc1(&Motor_AnglePID,Angle_Balance);      /* 计数得到位置式PID的增量数值 */		
//				
////				if(Para1<=0){	AIN2=1;AIN1=0;}
////				else {AIN2=0;AIN1=1;}
//				
//				/*【内环】电流部分 */
//				Para1=myabs(Para1);																		//内环限幅，限制上下限
//				if(Para1>=80)Para1=80;
//				Para1=1000;
				

//				Encoder_Bias=Encoder_Balance-Motor_PostionPID.SetPoint;	//===取相对值，变化量
				
//				if(Encoder_Bias>0){AIN2=0;AIN1=1;}									//判断方向，最终效果是Encoder_Bias=0;
//				else{	AIN2=1;AIN1=0;}
//				Encoder_Bias=myabEnter=Encoder_Bias;
//				Xianfu_Pwm();                        									 //===电机编码器限幅 (每个圆周循环增加2pai)s(Encoder_Bias);
//				Motor_PostionPID.SetPoint=Encoder_Balance;
//				Encoder_	
//				PID_SetPoint(&Motor_PostionPID,Para1);
					

	
					Encoder_Balance=Read_Encoder(4);             	        	//===更新编码器位置信息
					Para2=Inc_PID_Calc2(&Motor_PostionPID,Encoder_Balance);  /* 计数得到位置式PID的增量数值 */	
					Moto+=Para2;
					if(Para2<0){AIN2=0;AIN1=1;}													//判断方向，最终效果是Encoder_Bias=0;
					else{	AIN2=1;AIN1=0;}
					Xianfu_Pwm();                         									//===PWM限幅 
					Set_Pwm(Moto);                       								 //===赋值给PWM寄存器   
//				if(Turn_Off()==0)

				
				}
		//			Balance_Pwm =balance(Angle_Balance);                                          //===角度PD控制	
		//			if(Position_Target++>4)	Position_Pwm=Position(Encoder_Balance),Position_Target=0;     //===位置PD控制 25ms进行一次位置控制
		         
				
			}
			
		
		timer_count++;
		if(timer_count==10)
		{
			timer_count=0;
			timer_count2++;
			if(timer_count2==5)
			{
				timer_count2=0;
				Times_up=1;
				LED_TOGGLE;
			}		
		}

	}
}



