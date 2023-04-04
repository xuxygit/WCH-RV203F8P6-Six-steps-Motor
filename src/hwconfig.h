#ifndef _HWCONFIG_H_
#define _HWCONFIG_H_
/********************************** (C) COPYRIGHT *******************************
 * File Name          : hwconfig.h
 * Author             : XUXY
 * Version            : V1.0.0
 * Date               : 2023/03/06
 * Description        : system hardware initial file.
 * Copyright (c) 2023   Shenzhen NaXiang Co., Ltd.
 * Version            : V1.0.0
 * system initialization
 * MCU init, PWM,CMT,ADC,PIT, KBI and UART
 *****************************************************************************/

#include "ch32v20x_conf.h"
 /**********************************************************************
  *   in the HW board
  *	KEY = PD0
  * LED1 = PB3, LED2 = PB2, LED3 = PB1, LED4 = PB0
  *	TIM1_CH1 = PWM_UH, TIM1_CH1N = PWM_UL,
  *	TIM1_CH2 = PWM_VH, TIM1_CH2N = PWM_VL,
  *	TIM1_CH3= PWM_WH, TIM1_CH3N = PWM_WL,
  *
  **********************************************************************/
#define LED1 GPIO_Pin_3
#define LED2 GPIO_Pin_2
#define LED3 GPIO_Pin_1
#define LED4 GPIO_Pin_0
#define KEY  GPIO_Pin_6

#define PWMTIMER TIM1
#define CMTTIMER TIM2
#define PWM_FREQ (16000)                   /* PWM frequency - 16kHz */
#define PWMTIMER_Prescaler (4)
#define PWMTIMER_Period (SystemCoreClock/PWMTIMER_Prescaler/PWM_FREQ) 
#define PWM_DEAD_TIME (40)                /* dead time 1us = PWM_DEAD_TIME/20Mhz */
#define CTRL_LOOP_FREQ (1000)              /* Control loop frequency */

#define BEMF_U ADC_Channel_4
#define BEMF_V ADC_Channel_5
#define BEMF_W ADC_Channel_7
#define IDcbus ADC_Channel_8
#define VREFH (5) // MCU Vdd=Vrefh=5.0v

void LedInit(void);
void KeyInit(void);
void CMTTimerInit(uint16_t);
void PWMTimerInit(void);
void ADCInit(void);
uint16_t GetADCVal(uint8_t ch);
void GPIO_PinToggle(uint32_t pin);
void SetPWMDutyCycle(uint8_t induty);
void SetPWMPhase(uint8_t);
uint8_t NextSector(uint8_t);
void SetCMTPeriod(uint16_t);


#endif