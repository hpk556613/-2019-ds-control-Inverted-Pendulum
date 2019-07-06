#ifndef __USRAT3_H
#define __USRAT3_H 
/**************************************************************************
���ߣ�Mini Balance 
�Ա����̣�http://shop114407458.taobao.com/
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


	  	
extern u8 USART_RX_BUF[64];     //���ջ���,���63���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8 USART_RX_STA;         //����״̬���	

//����봮���жϽ��գ��벻Ҫע�����º궨��
#define EN_USART1_RX //ʹ�ܴ���1����
void uart_init(u32 pclk2,u32 bound);
void uart3_init(u32 pclk2,u32 bound);

void USART3_IRQHandler(void);


#endif

