#include "control.h"		


PIDStruct Motor_AnglePID;
PIDStruct Motor_PostionPID;



void Swing(void)									//�ڶ����߼�
{	
	if(Times_up2==1)
	{
			Times_up2=0;
			Motor_PostionPID.SetPoint=2000;
			Encoder_Balance=Read_Encoder(4);             	        	//===���±�����λ����Ϣ
			Para2=Loc_PID_Calc2(&Motor_PostionPID,Encoder_Balance);  /* �����õ�λ��ʽPID��������ֵ */	
			Moto=Para2;
			if(Para2<0){AIN2=0;AIN1=1;}													//�жϷ�������Ч����Encoder_Bias=0;
			else{	AIN2=1;AIN1=0;}
			Moto=Xianfu_Pwm(Moto);                         				//===PWM�޷� 
			Set_Pwm(Moto);  
//		if((Angle_Balance>950)&&(Angle_Balance<2850))
//		{
//			PIDStart_Check();
//		}
		if((Angle_Balance>2850)&&(Angle_Balance<3250))
		{
			Check_start=1;
		}
	}
}




/**
  * @brief  PID�ṹ���ʼ������
  * @param  pidStuc PID�ṹ��ָ��
  * @param  SetPoint Ŀ��ֵ
  * @retval ��
  */
/**************PID������ʼ��********************************/
void PID_Init(PIDStruct * pidStuc, float kp, float ki, float kd)
{
	pidStuc->Proportion = kp;
	pidStuc->Integral = ki;
	pidStuc->Derivative = kd;
	pidStuc->LastError = 0;
	pidStuc->PrevError = 0;
	pidStuc->LastErrorSum=0;
}

/**************************************************************************
�������ܣ�λ��PD���� 
��ڲ�����������
����  ֵ��λ�ÿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int Loc_PID_Calc1(PIDStruct * pidStuc,int NextPoint)
{
  int iError,iIncpid;                                 //��ǰ���
  iError=NextPoint-pidStuc->SetPoint;                    //��������
  iIncpid=(pidStuc->Proportion * iError)                 //E[k]��
           +(pidStuc->Derivative * pidStuc->PrevError);  //E[k-2]-E[k-1]��
              
  pidStuc->PrevError=pidStuc->LastError;                    //�洢�������´μ���
  pidStuc->LastError=iError;
  return(iIncpid);                                    //��������ֵ
}


/**************************************************************************
�������ܣ�λ��ʽPD���� 
��ڲ�����������
����  ֵ��λ�ÿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int Loc_PID_Calc2(PIDStruct * pidStuc,int NextPoint)
{
  int iError,iIncpid;                                 //��ǰ���
  iError=pidStuc->SetPoint-NextPoint;
  pidStuc->LastErrorSum+=iError;
  iIncpid=(pidStuc->Proportion * iError)                 //E[k]��
              +(pidStuc->Integral * pidStuc->LastErrorSum)     //E[k-1]��
              +(pidStuc->Derivative * pidStuc->PrevError);  //E[k-2]��
	
	pidStuc->PrevError=pidStuc->LastError;                    //�洢�������´μ���
  pidStuc->LastError=iError;
              
  return(iIncpid);   
}

/**
  * @brief  PID�趨Ŀ��ֵ
  * @param  pidStuc PID�ṹ��ָ��
  * @param  setPoint Ŀ��ֵ
  * @retval ��
  */

void PID_SetPoint(PIDStruct * pidStuc, int SetPoint)
{
	pidStuc->SetPoint = SetPoint;
}




/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off(void)
{
		u8 temp; 
		if((Angle_Balance<(ZHONGZHI-800)||Angle_Balance>(ZHONGZHI+800))==1) //��ص�ѹ���ͣ��رյ��
		{	      			
		temp=1;                                            
		AIN1=0;                                            
		AIN2=0;
		}
		else
		temp=0;
		return temp;			
}
	

