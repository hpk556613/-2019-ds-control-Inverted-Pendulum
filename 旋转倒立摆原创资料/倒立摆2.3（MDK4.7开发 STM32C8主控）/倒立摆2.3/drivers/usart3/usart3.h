#ifndef __USRAT3_H
#define __USRAT3_H 
/**************************************************************************
作者：Mini Balance 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
#include "main.h"
#define GPIO_CR_RESET                 (uint32_t)0x44444444 
#define GPIO_CR_MODE_INPUT            (uint32_t)0x00000000
#define GPIO_CR_MODE_2MHz             (uint32_t)0x22222222
#define GPIO_CR_MODE_10MHz            (uint32_t)0x11111111
#define GPIO_CR_MODE_50MHz            (uint32_t)0x33333333

#define GPIO_CR_GP_PUSHPULL           (uint32_t)0x00000000
#define GPIO_CR_GP_OPENDRAIN          (uint32_t)0x44444444 
#define GPIO_CR_OUT_PP2MHz            (GPIO_CR_MODE_2MHz | GPIO_CR_GP_PUSHPULL)
#define GPIO_CR_OUT_PP50MHz           (GPIO_CR_MODE_50MHz | GPIO_CR_GP_PUSHPULL)

   
#define GPIO_CR_AFO_PUSHPULL           (uint32_t)0x88888888
#define GPIO_CR_AFO_OPENDRAIN          (uint32_t)0xcccccccc 

#define GPIO_CR_AFOUT_PP2MHz          (GPIO_CR_MODE_2MHz | GPIO_CR_AFO_PUSHPULL)  // ??????,2MHz
#define GPIO_CR_AFIN_FLOAT            (uint32_t)0x44444444  // ??????
#define GPIO_CR_AFIN_PULLDOWN         (uint32_t)0x88888888  // ????????


#define USART_CR1_REST                (uint32_t)0x00000000
#define USART_CR2_REST                (uint32_t)0x00000000
#define USART_CR3_REST                (uint32_t)0x00000000


	  	
extern u8 USART_RX_BUF[64];     //接收缓冲,最大63个字节.末字节为换行符 
extern u8 USART_RX_STA;         //接收状态标记	

//如果想串口中断接收，请不要注释以下宏定义
#define EN_USART1_RX //使能串口1接收
void uart_init(u32 pclk2,u32 bound);
void uart3_init(u32 pclk2,u32 bound);

void USART3_IRQHandler(void);


#endif

