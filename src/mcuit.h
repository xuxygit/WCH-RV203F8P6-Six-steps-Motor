/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v20x_it.h
 * Author             : XUXY
 * Version            : V1.0.0
 * Date               : 2023/03/06
 * Description        : This file contains the headers of the interrupt handlers.
 * Copyright (c) 2023   Shenzhen NaXiang Co., Ltd.
 *******************************************************************************/
#ifndef __MCU_IT_H
#define __MCU_IT_H

#include "ch32v20x_conf.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI9_5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void ADC1_IRQHandler(void)__attribute__((interrupt("WCH-Interrupt-fast")));

#endif /* __MCU_IT_H */

