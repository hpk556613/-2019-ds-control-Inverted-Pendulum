#include "main.h"
/**************************************************************************
���ߣ�Mini Balance 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/
u8 Way_Angle=1;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲�
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right; //����ң����صı���
u8 Flag_Stop=1,Flag_Show=1;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_Left,Encoder_Right;             //���ұ��������������
int Moto1,Moto2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int adc;                            //��ʾ�¶�
int Voltage;                                //��ص�ѹ������صı���
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������

/**************************************************************************
�������ܣ������� ��ʼ��ϵͳ������
��    �ߣ�Mini Balance
**************************************************************************/
int main(void)
{
	SystemInit();                   //=====ϵͳ��ʼ��
	delay_init(72);                 //=====��ʱ����
	usart1_init();                  //=====����1��ʼ�� �����ʣ�115200
	uart3_init(72,9600);            //=====����3��ʼ�� �����ʣ�9600
	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
  led_init();                     //=====LED��ʼ��
	KEY_Init();                     //=====������ʼ��
	Adc_Init();	                    //=====��ʼ��ADCģ��
	MiniBalance_PWM_Init(7199,0);   //=====��ʼ��PWM
	OLED_Init();	                  //=====��ʼ��OLED  
	Encoder_Init();                 //=====��ʼ��������1
	delay_ms(200);                  //=====��ʱ�ȴ��ȶ�		
	Timer1_Init(49,7199);           //=====5MS��һ���жϷ����� �жϷ�������minibalance.c����
  while(1)
  { 

					if(1==Flag_Show)		oled_show(); //===��ʾ����
					else	              DataScope(); //===��ʾ���ر� ����λ��			
	}	
}

