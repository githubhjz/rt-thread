/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 */

#include <rthw.h>
#include <rtthread.h>

#include "stm32f4xx_hal.h"
#include "board.h"

/**
 * @addtogroup STM32
 */

/*@{*/

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
//#ifdef  VECT_TAB_RAM
//	/* Set the Vector Table base location at 0x20000000 */
//	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
//#else  /* VECT_TAB_FLASH  */
//	/* Set the Vector Table base location at 0x08000000 */
//	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
//#endif

//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}



/**
 * This function will initial STM32 board.
 */
void rt_hw_board_init()
{
	/* NVIC Configuration */
	NVIC_Configuration();

	rt_hw_usart_init();
#ifdef RT_USING_CONSOLE
	rt_console_set_device(CONSOLE_DEVICE);
#endif
}

/*@}*/
