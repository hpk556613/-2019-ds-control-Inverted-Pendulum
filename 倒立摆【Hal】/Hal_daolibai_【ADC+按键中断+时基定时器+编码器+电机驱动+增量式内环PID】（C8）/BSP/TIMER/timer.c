#include "timer.h"


/**
  * ��������: ������ģʽ�¶�ʱ���Ļص�����
  * �������: htim����ʱ�����
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim1))
	{
		Angle_Balance=Get_Adc_Average(3,15);               //===������̬			
		
		if(Position_Key==1)																//����USER�����ſ�ʼ����
		{
			if(Times_up==1)
			{
				Times_up=0;

				/* ���⻷���ǶȲ���*/
								
//				Error_Target=Angle_Balance-Motor_AnglePID.SetPoint;
//				Para1=Inc_PID_Calc1(&Motor_AnglePID,Angle_Balance);      /* �����õ�λ��ʽPID��������ֵ */		
//				
////				if(Para1<=0){	AIN2=1;AIN1=0;}
////				else {AIN2=0;AIN1=1;}
//				
//				/*���ڻ����������� */
//				Para1=myabs(Para1);																		//�ڻ��޷�������������
//				if(Para1>=80)Para1=80;
//				Para1=1000;
				

//				Encoder_Bias=Encoder_Balance-Motor_PostionPID.SetPoint;	//===ȡ���ֵ���仯��
				
//				if(Encoder_Bias>0){AIN2=0;AIN1=1;}									//�жϷ�������Ч����Encoder_Bias=0;
//				else{	AIN2=1;AIN1=0;}
//				Encoder_Bias=myabEnter=Encoder_Bias;
//				Xianfu_Pwm();                        									 //===����������޷� (ÿ��Բ��ѭ������2pai)s(Encoder_Bias);
//				Motor_PostionPID.SetPoint=Encoder_Balance;
//				Encoder_	
//				PID_SetPoint(&Motor_PostionPID,Para1);
					

	
					Encoder_Balance=Read_Encoder(4);             	        	//===���±�����λ����Ϣ
					Para2=Inc_PID_Calc2(&Motor_PostionPID,Encoder_Balance);  /* �����õ�λ��ʽPID��������ֵ */	
					Moto+=Para2;
					if(Para2<0){AIN2=0;AIN1=1;}													//�жϷ�������Ч����Encoder_Bias=0;
					else{	AIN2=1;AIN1=0;}
					Xianfu_Pwm();                         									//===PWM�޷� 
					Set_Pwm(Moto);                       								 //===��ֵ��PWM�Ĵ���   
//				if(Turn_Off()==0)

				
				}
		//			Balance_Pwm =balance(Angle_Balance);                                          //===�Ƕ�PD����	
		//			if(Position_Target++>4)	Position_Pwm=Position(Encoder_Balance),Position_Target=0;     //===λ��PD���� 25ms����һ��λ�ÿ���
		         
				
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



