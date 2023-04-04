
/********************************** (C) COPYRIGHT *******************************
 * File Name          : statemachine.c
 * Author             : XUXY
 * Version            : V1.0.0
 * Date               : 2023/03/06
 * Description        : files of motor state machine.
 * Copyright (c) 2023   Shenzhen NaXiang Co., Ltd.
 * Version            : V1.0.0 *
 * The main function are the following:
 * 1) MotorInit()     -> Init the main variables for motor driving from Motorparam.h
 * 2) MotorReset()    -> Reset all variables used for 6Step control algorithm
 * 3) MotorStart()       -> Start the Motor
 * 4) MotorStop()       	-> Stop the Motor
 * 5) MotorAlign()			-> Alignment Motor
 * 6) MotorOpenLoop()		-> Openloop drive Motor
 * 7) MotorRun()			-> Run Motor
 * 8) MotorFault()			-> Found faults
 * 9) MotorZCDetecting()	-> Zero crossing detected
 * 10) MotorSetSpeed()		-> set motor mechanic speed in RPM
 * 11) MotorGeteSpeed()	-> get motor speed in CMT period
 * 12) MotorGetmSpeed()	-> get motor speed in mechanic RPM
 * run in state machine loop
 *****************************************************************************/

#include "motoparam.h"
#include "motoctrl.h"
#include "statemachine.h"
#include "hwconfig.h"
#include "mcuit.h"
 //#include "debug.h"

extern Motor_Ctrl_t motor_ctrl;
SysStatus_t status = START;
Flags_t flags = NONE;
uint16_t rounds = 0;
uint32_t counting = 0;

void StateMachine(void)
{
    switch (status)
    {
    case INIT:
        {
            MotorInit();
            status = START;
        }
        break;

    case STOP:
        {
            MotorStop();
            motor_ctrl.round_count = 0;
            motor_ctrl.PWM_step_count = 0;
            status = STOP;
        }
        break;
    case START:
        {
            MotorStart();
            status = ALIGNMENT;
            counting = 0;
        }
        break;

        /* Force motor to a fix position, ready for open loop drive*/
    case ALIGNMENT:
        {
            if (counting == 0)
                MotorAlign();

            counting++;
            if (counting > 32) // 2ms for 16khz pwm;
            {
                SetPWMPhase(0);
                flags = ALIGN_OK;
                status = OPENLOOP;
                counting = 0;
                rounds = 0;
            };
        }
        break;

        /*
            Force motor run and ramup to a threshold speed, for zero  crossing detecting
            if ZCT fail after some round, add more power via dutycycle, try again
        */
    case OPENLOOP:
        {

            if (rounds == 0)
            {
                MotorOpenLoop();			   // set open loop duty cycle and period
                motor_ctrl.CMT_step_count = 0; // clear step_count;
            }

            /*	Ramping up, speed accelerate to open loop threshold */
            // every 6 steps * pole pairs, motor run 1 round
            motor_ctrl.round_count = motor_ctrl.CMT_step_count % (6 * NUM_POLE_PAIRS);
            if (!(motor_ctrl.round_count))  // adjust speed every motor run 1 round
            {
                rounds++;
                SpeedAccelerate(OPENLOOP_SPEED, rounds);
                //				motor_ctrl.CMT_period -= 10;
                //			SetCMTPeriod(motor_ctrl.CMT_period);
            }

            if (rounds > 200) // for the motor, 1200rpm as ZCD speed
            {
                status = SPIN;
                counting = 0;
            }
        }
        break;

        /*	Zero Crossing detecting, run the motor at right CMT period
                adjust motor speed to desired value*/
    case SPIN:
        {
            // keep running at the speed wait for zero crossing detected
            counting++;
            if (counting > 32000) // 2s for 16khz pwm;
            {
                if (flags == ZCDETECT_OK)
                {
                    SetPWMDutyCycle(60); // TODO: run at low speed, maybe too low power
                    GPIO_PinToggle(LED3); // TODO:just for debug, show ZC;
                    status = RUN;
                }
                else
                {
                    status = FAULT;
                    flags = ZCDETECT_FAIL;
                    GPIO_PinToggle(LED4); // TODO:just for debug, show error;
                }

                counting = 0;
            }
        }
        break;

    case RUN:
        {
            // counting++;
            // if((counting<<7) >=motor_ctrl.ZC_lead_time))	// as PWMTIMER 2^7 times run over CMTTIMER
            if (CMTTIMER->CNT >= (motor_ctrl.ZC_lead_time))
            {
                SetPWMPhase(motor_ctrl.phase_sector);
                MotorRun();
                counting = 0;
            }
            status = SPIN;
            counting = 0;
        }
        break;
    case FAULT:
        MotorFault();
        break;
    default:
        MotorInit();
        break;
    }
}

/*******************************************************************************
 * Initiation all motor control parameters;
 * Set Motor state machine to INIT;
 *******************************************************************************/
void MotorInit(void)
{
    uint8_t i;

    status = INIT;

    motor_ctrl.PWM_period = PWMTIMER_Period;
    motor_ctrl.CMT_period = CMT_PERIOD;
    motor_ctrl.phase_sector = 0;
    motor_ctrl.duty_cycle = 50; // for bipolar control, 50% is balanced, motor should not move

    motor_ctrl.round_count = 0;
    motor_ctrl.PWM_step_count = 0;
    motor_ctrl.CMT_step_count = 0;
    motor_ctrl.demag_time = CMT_PERIOD >> 2;
    motor_ctrl.ZC_lead_time = 0;
    motor_ctrl.ZC_count = 0;

    /* Zero crossing values*/
    for (i = 0; i < 3; i++)
    {
        motor_ctrl.Bemf_volt[ i ] = 0x55; // just no zero value in volt
        motor_ctrl.ZC_time[ i ] = 0x7ff;
    }
    motor_ctrl.Bemf_volt_index = 0;
    motor_ctrl.Bemf_sum = 0;
    motor_ctrl.adc_channel = BEMF_W;

    /* get VREFH for Vdbus ref */
    ADC_RegularChannelConfig(ADC2, ADC_Channel_Vrefint, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC2, ENABLE);
    while (!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC));
    motor_ctrl.Vrefd = ADC_GetConversionValue(ADC2);
    motor_ctrl.Vrefd = (motor_ctrl.Vrefd) >> 11;     // 12bits, if VDD=5.0v, then full VREF=5v;
    motor_ctrl.Vrefd *= VREFH;  // per 1 volt per bits means Volt;
    ADC_ClearFlag(ADC2, ADC_FLAG_EOC);

    /* speed PID control para*/
    motor_ctrl.accel = MINIMUM_ACC;
    motor_ctrl.KP = KP_GAIN_VM;
    motor_ctrl.KI = KI_GAIN_VM;
    motor_ctrl.KPD = KP_DIV_VM;
    motor_ctrl.KID = KI_DIV_VM;
    motor_ctrl.integral_sum = 0;
    motor_ctrl.speed_fb = 0;
    motor_ctrl.speed_e = 0;
    motor_ctrl.speed_mech = 0;

    GPIO_SetBits(GPIOB, LED3);
    GPIO_SetBits(GPIOB, LED4);
}

void MotorReset(void)
{
    MotorInit();
    status = ALIGNMENT;
}

/******************************************************************************
 * switch MOSFET PWM driver off;
 * disable CMTTIMER interrupt;
 ******************************************************************************/
void MotorStop(void)
{
    /* 	PWMTIMER driver OFF; */
    SetPWMPhase(7);
    /* 	Disable CMTTIMER,ADC and RTC ISR; */
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);
    //	NVIC_DisableIRQ(TIM2_IRQn);
    //	NVIC_DisableIRQ(ADC1_2_IRQn);
}


void MotorStart(void)
{
    status = START;
    motor_ctrl.PWM_step_count = 0;
    motor_ctrl.duty_cycle = 65; // TODO: maybe start from >60 ?
    SetPWMDutyCycle(motor_ctrl.duty_cycle);
    motor_ctrl.CMT_period = 0x7fff;
    SetCMTPeriod(motor_ctrl.CMT_period);

    /* enable CMT timer for phase switch*/
    TIM_ITConfig(CMTTIMER, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM2_IRQn);

}

/******************************************************************************
 * switch MOSFET PWM driver to alignment sector;
 * set PWM duty cycle to alignment duty cycle;
 ******************************************************************************/
void MotorAlign(void)
{

    SetPWMPhase(5);
    motor_ctrl.duty_cycle = DUTY_CYCLE_ALIGN;
    SetPWMDutyCycle(motor_ctrl.duty_cycle);
}

/*******************************************************************************
 * switch MOSFET PWM driver sector;
 * set PWM dutycycle to openloop dutycycle;
 * force motor run at per-define speed sloop;
 *******************************************************************************/
void MotorOpenLoop()
{

    motor_ctrl.duty_cycle = DUTY_CYCLE_OP + 10;
    SetPWMDutyCycle(motor_ctrl.duty_cycle);

    motor_ctrl.CMT_period = 0x9ff;
    SetCMTPeriod(motor_ctrl.CMT_period);
}

void MotorRun(void)
{
    status = RUN;
    rounds = 0;
    counting = 0;
    motor_ctrl.ZC_count = 0;

    // change duty cycle and CMT period as speed need;
    SetPWMDutyCycle(motor_ctrl.duty_cycle);
    SetCMTPeriod(motor_ctrl.CMT_period);
}

void MotorFault(void)
{

    if (flags == OPENLOOP_FAIL)
    {
        MotorStop();
    }
    if (flags == OVERCURRENT)
    {
        // count = motor_ctrl.Ms_count;
        // if ((motor_ctrl.Ms_count - count) > 500) // about 0.5s high curent
        MotorReset();
    }
}

/*******************************************************************************
 * Zero Cross point detecting;
 *******************************************************************************/
uint8_t MotorZCDetecting(void)
{

    uint16_t bemfv;
    uint8_t phase;

    phase = motor_ctrl.phase_sector;


    motor_ctrl.Bemf_sum = 0;
    /*
        for (i = 0; i < 3; i++)
            motor_ctrl.Bemf_sum += motor_ctrl.Bemf_volt[i];
        bemfv = motor_ctrl.Bemf_sum / 3;
    */
    motor_ctrl.Bemf_sum = motor_ctrl.Bemf_volt[ 0 ] + motor_ctrl.Bemf_volt[ 1 ];
    bemfv = motor_ctrl.Bemf_sum / 2;

    if (bemfv < BEMF_THRESHOLD)
    {
        motor_ctrl.ZC_time[ motor_ctrl.ZC_count ] = CMTTIMER->CNT; // record the event time
        motor_ctrl.ZC_count++;
        motor_ctrl.ZC_count %= 6;	// record 6 ZC time 
        phase = motor_ctrl.phase_sector;

    }

    if ((motor_ctrl.phase_sector - phase) == 1) // phase_sctor increase in CMTTimer ISR
    {
        motor_ctrl.CMT_period = CMTTIMER->ATRLR - motor_ctrl.ZC_time[ motor_ctrl.ZC_count - 1 ] \
            + motor_ctrl.ZC_time[ motor_ctrl.ZC_count ];
        motor_ctrl.ZC_lead_time = motor_ctrl.CMT_period / 2;
        flags = ZCDETECT_OK; // set flags and change status
        //		SetCMTPeriod(motor_ctrl.CMT_period);
        status = RUN;
        return TRUE;
    }
    return FALSE;
}
