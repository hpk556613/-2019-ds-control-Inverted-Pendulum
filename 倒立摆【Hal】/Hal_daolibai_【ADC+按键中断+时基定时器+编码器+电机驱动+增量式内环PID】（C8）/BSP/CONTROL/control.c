#include "control.h"		


PIDStruct Motor_AnglePID;
PIDStruct Motor_PostionPID;


/**
  * @brief  PID结构体初始化函数
  * @param  pidStuc PID结构体指针
  * @param  SetPoint 目标值
  * @retval 无
  */
/**************PID参数初始化********************************/
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
函数功能：位置PD控制 
入口参数：编码器
返回  值：位置控制PWM
作    者：平衡小车之家
**************************************************************************/
int Inc_PID_Calc1(PIDStruct * pidStuc,int NextPoint)
{
  int iError1,iIncpid1;                                 //当前误差
  iError1=NextPoint-pidStuc->SetPoint;                    //增量计算
  iIncpid1=(pidStuc->Proportion * iError1)                 //E[k]项
           +(pidStuc->Derivative * pidStuc->PrevError);  //E[k-2]-E[k-1]项
              
  pidStuc->PrevError=pidStuc->LastError;                    //存储误差，用于下次计算
  pidStuc->LastError=iError1;
  return(iIncpid1);                                    //返回增量值
}


/**************************************************************************
函数功能：增量式PD控制 
入口参数：编码器
返回  值：位置控制PWM
作    者：平衡小车之家
**************************************************************************/
int Inc_PID_Calc2(PIDStruct * pidStuc,int NextPoint)
{
   int iError,iIncpid2;                                 //当前误差
  iError=pidStuc->SetPoint-NextPoint;
  pidStuc->LastErrorSum+=iError;
  iIncpid2=(pidStuc->Proportion * iError)                 //E[k]项
              +(pidStuc->Integral * pidStuc->LastErrorSum)     //E[k-1]项
              +(pidStuc->Derivative * pidStuc->PrevError);  //E[k-2]项
              
  return(iIncpid2);   
}



/**
  * @brief  PID设定目标值
  * @param  pidStuc PID结构体指针
  * @param  setPoint 目标值
  * @retval 无
  */

void PID_SetPoint(PIDStruct * pidStuc, int SetPoint)
{
	pidStuc->SetPoint = SetPoint;
}




/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(void)
{
		u8 temp; 
		if((Angle_Balance<(ZHONGZHI-800)||Angle_Balance>(ZHONGZHI+800))==1) //电池电压过低，关闭电机
		{	      			
		temp=1;                                            
		AIN1=0;                                            
		AIN2=0;
		}
		else
		temp=0;
		return temp;			
}
	

