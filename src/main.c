
/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "debug.h"
#include "hwconfig.h"
#include "statemachine.h"

extern uint8_t i;
/*****************************************************************************
* system initialization
* MCU init, PWM,CMT,ADC,PIT, KBI and UART
* App variable initialization
*****************************************************************************/
int main(void)
{
    uint8_t i = 0;


    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();

    // init system mcu function block, LED, KEY, TIM and ADC
    LedInit();
    KeyInit();
    PWMTimerInit();
    CMTTimerInit(3600);
    ADCInit();


    while (1)
    {
        Delay_Ms(250);
        GPIO_PinToggle(LED1);
        //  GPIO_WriteBit(GPIOC, GPIO_Pin_13, (i == 0) ? (i = Bit_SET) : (i = Bit_RESET));
    }
}
