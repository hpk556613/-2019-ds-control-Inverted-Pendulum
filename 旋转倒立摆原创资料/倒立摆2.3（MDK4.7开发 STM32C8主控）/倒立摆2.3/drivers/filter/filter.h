#ifndef __FILTER_H
#define __FILTER_H

/**************************************************************************
×÷Õß£ºMini Balance 
ÌÔ±¦µêÆÌ£ºhttp://shop114407458.taobao.com/
**************************************************************************/
extern float angle, angle_dot; 	
void Kalman_Filter(float Accel,float Gyro);		
void Yijielvbo(float angle_m, float gyro_m);
#endif
