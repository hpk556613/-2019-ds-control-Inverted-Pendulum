#include "control.h"		


PIDStruct Motor_AnglePID;
PIDStruct Motor_PostionPID;


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
int Inc_PID_Calc1(PIDStruct * pidStuc,int NextPoint)
{
  int iError1,iIncpid1;                                 //��ǰ���
  iError1=NextPoint-pidStuc->SetPoint;                    //��������
  iIncpid1=(pidStuc->Proportion * iError1)                 //E[k]��
           +(pidStuc->Derivative * pidStuc->PrevError);  //E[k-2]-E[k-1]��
              
  pidStuc->PrevError=pidStuc->LastError;                    //�洢�������´μ���
  pidStuc->LastError=iError1;
  return(iIncpid1);                                    //��������ֵ
}


/**************************************************************************
�������ܣ�����ʽPD���� 
��ڲ�����������
����  ֵ��λ�ÿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int Inc_PID_Calc2(PIDStruct * pidStuc,int NextPoint)
{
   int iError,iIncpid2;                                 //��ǰ���
  iError=pidStuc->SetPoint-NextPoint;
  pidStuc->LastErrorSum+=iError;
  iIncpid2=(pidStuc->Proportion * iError)                 //E[k]��
              +(pidStuc->Integral * pidStuc->LastErrorSum)     //E[k-1]��
              +(pidStuc->Derivative * pidStuc->PrevError);  //E[k-2]��
              
  return(iIncpid2);   
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
	

