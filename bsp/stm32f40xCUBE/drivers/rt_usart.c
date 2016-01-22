/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 * 2012-02-08     aozima       update for F4.
 */

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

/*
 * Use UART1 as console output and finsh input
 * interrupt Rx and poll Tx (stream mode)
 *
 * Use UART2 with interrupt Rx and poll Tx
 * Use UART3 with DMA Tx and interrupt Rx -- DMA channel 2
 *
 * USART DMA setting on STM32
 * USART1 Tx --> DMA Channel 4
 * USART1 Rx --> DMA Channel 5
 * USART2 Tx --> DMA Channel 7
 * USART2 Rx --> DMA Channel 6
 * USART3 Tx --> DMA Channel 2
 * USART3 Rx --> DMA Channel 3
 */

#ifdef RT_USING_UART1
struct stm32_serial_int_rx uart1_int_rx;
struct stm32_serial_device uart1 =
{
	USART1,
	&uart1_int_rx,
	RT_NULL
};
struct rt_device uart1_device;
#endif


/* USART1_REMAP = 0 */
//#define UART1_GPIO_TX		GPIO_Pin_9
//#define UART1_TX_PIN_SOURCE GPIO_PinSource9
//#define UART1_GPIO_RX		GPIO_Pin_10
//#define UART1_RX_PIN_SOURCE GPIO_PinSource10
//#define UART1_GPIO			GPIOA
//#define UART1_GPIO_RCC      RCC_AHB1Periph_GPIOA
//#define RCC_APBPeriph_UART1	RCC_APB2Periph_USART1
//#define UART1_TX_DMA		DMA1_Channel4
//#define UART1_RX_DMA		DMA1_Channel5





static void RCC_Configuration(void)
{
//#ifdef RT_USING_UART1
//	/* Enable USART2 GPIO clocks */
//	RCC_AHB1PeriphClockCmd(UART1_GPIO_RCC, ENABLE);
//	/* Enable USART2 clock */
//	RCC_APB2PeriphClockCmd(RCC_APBPeriph_UART1, ENABLE);
//#endif
}

static void GPIO_Configuration(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;

//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

//#ifdef RT_USING_UART1
//	/* Configure USART1 Rx/tx PIN */
//	GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX | UART1_GPIO_TX;
//	GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

//    /* Connect alternate function */
//    GPIO_PinAFConfig(UART1_GPIO, UART1_TX_PIN_SOURCE, GPIO_AF_USART1);
//    GPIO_PinAFConfig(UART1_GPIO, UART1_RX_PIN_SOURCE, GPIO_AF_USART1);
//#endif
}

static void NVIC_Configuration(void)
{
//	NVIC_InitTypeDef NVIC_InitStructure;

//#ifdef RT_USING_UART1
//	/* Enable the USART1 Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//#endif

}

static void DMA_Configuration(void)
{
}

volatile USART_TypeDef * uart2_debug = USART2;
/*
 * Init all related hardware in here
 * rt_hw_serial_init() will register all supported USART device
 */
void rt_hw_usart_init()
{
//USART_InitTypeDef USART_InitStructure;

//RCC_Configuration();

//GPIO_Configuration();

//NVIC_Configuration();

//DMA_Configuration();

///* uart init */
//#ifdef RT_USING_UART1
////	USART_InitStructure.USART_BaudRate = 115200;
////	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
////	USART_InitStructure.USART_StopBits = USART_StopBits_1;
////	USART_InitStructure.USART_Parity = USART_Parity_No;
////	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
////	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
////	USART_Init(USART1, &USART_InitStructure);

////	/* register uart1 */
////	rt_hw_serial_register(&uart1_device, "uart1",
////		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
////		&uart1);

////	/* enable interrupt */
////	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//#endif
}
