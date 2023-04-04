/********************************** (C) COPYRIGHT *******************************
 * File Name          : mcuit.c
 * Author             : XUXY
 * Version            : V1.0.0
 * Date               : 2023/03/06
 * Description        : Main Interrupt Service Routines.
 * Copyright (c) 2023   Shenzhen NaXiang Co., Ltd.
 * Version            : V1.0.0
 *******************************************************************************/
#include "mcuit.h"
#include "motoctrl.h"
#include "hwconfig.h"

extern Motor_Ctrl_t motor_ctrl;
extern const PWMCtrl_t CMTTable[];
extern uint8_t sector;
static uint8_t key_push_times = 0;

/*
* Channel change sequence as Motor phase sector position,
* sector= 0~5 as six step, sector 6 for alignment and sector 7 for stop;
*/
void TIM2_IRQHandler(void)
{

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查TIM2中断是否发生。
    {
        /*
            TIM_CCxCmd(TIM1, TIM_Channel_1, CMTTable[ sector ].Channel_1);
            TIM_CCxNCmd(TIM1, TIM_Channel_1, CMTTable[ sector ].Channel_1N);

            TIM_CCxCmd(TIM1, TIM_Channel_2, CMTTable[ sector ].Channel_2);
            TIM_CCxNCmd(TIM1, TIM_Channel_2, CMTTable[ sector ].Channel_2N);

            TIM_CCxCmd(TIM1, TIM_Channel_3, CMTTable[ sector ].Channel_3);
            TIM_CCxNCmd(TIM1, TIM_Channel_3, CMTTable[ sector ].Channel_3N);

            sector++;
            sector %= 5;
        */
        //  switch to next phase
        SetPWMPhase(motor_ctrl.phase_sector);
        motor_ctrl.phase_sector = NextSector(motor_ctrl.phase_sector);

        // set next ADC channel
        ADC_InjectedChannelConfig(ADC1, CMTTable[ motor_ctrl.phase_sector ].ADC_Channel, 1, ADC_SampleTime_28Cycles5);

    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);    //清除TIM2的中断挂起位。
}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line6) == SET)  //EXTI_GetITStatus用来获取中断标志位状态，如果EXTI线产生中断则返回SET，否则返回RESET
    {
        key_push_times %= 5;
        key_push_times++;

        switch (key_push_times)
        {
        case 1:
            GPIO_PinToggle(LED1);
            // MotorSetSpeed(TARGET_SPEED25);
            // MotorStart();
            break;
        case 2:
            GPIO_PinToggle(LED1);
            GPIO_PinToggle(LED2);
            // MotorSetSpeed(TARGET_SPEED30);
            break;
        case 3:
            GPIO_PinToggle(LED2);
            GPIO_PinToggle(LED3);
            // MotorSetSpeed(TARGET_SPEED35);
            break;
        case 4:
            GPIO_PinToggle(LED3);
            GPIO_PinToggle(LED4);
            // MotorSetSpeed(TARGET_SPEED40);
            break;
        case 5:
            GPIO_PinToggle(LED4);
            // MotorStop();
            //			PowerOff();
            break;
        default:
            GPIO_WriteBit(GPIOA, LED1, 1);
            GPIO_WriteBit(GPIOA, LED2, 1);
            GPIO_WriteBit(GPIOA, LED3, 1);
            GPIO_WriteBit(GPIOA, LED4, 1);
        }

        EXTI_ClearFlag(EXTI_Line6);
    }
}


/*******************************************************************************
* Function Name  : ADC1_2_IRQHandler
* Description    : This function handles ADC1 injection conversion exception.
* Input          : None
* Return         : None
*******************************************************************************/
void ADC1_IRQHandler(void)
{

    if (ADC_GetITStatus(ADC1, ADC_IT_JEOC))
        motor_ctrl.Bemf_volt_nopow = ADC_GetInjectedConversionValue(ADC1, CMTTable[ motor_ctrl.phase_sector ].ADC_Channel);

    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);

}
