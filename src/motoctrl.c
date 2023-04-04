/********************************** (C) COPYRIGHT *******************************
 * File Name          : motoctrl.c
 * Author             : XUXY
 * Version            : V1.0.0
 * Date               : 2023/03/06
 * Description        : Motor control file.
 * Copyright (c) 2023   Shenzhen NaXiang Co., Ltd.
 * Version            : V1.0.0 *
 *****************************************************************************/
#include "motoctrl.h"
#include "motoparam.h"
#include "hwconfig.h"


extern Motor_Ctrl_t motor_ctrl;
extern Flags_t flags;

/******************************************************************************
*  Calc the root of input, Using Newton way;
*  Since the input is max 0xFFFF, and only inter output need, take easy way;
*******************************************************************************/
float MinSqrt(uint16_t in)
{
    /*
        uint16_t rootlast,rootnew;

        rootlast = 256;

        for(;;)
        {
            rootnew = (rootlast + in/rootlast) / 2;
            if((rootlast - rootnew) > 1)
                rootlast = rootnew;
            else
                return ( rootnew );
        }
    */
    float root[ 32 ] = {
        1.000, 1.414, 1.732, 2.000, 2.236, 2.449, 2.646, 2.828, 3.000, 3.162,
        3.317, 3.464, 3.606, 3,742, 3,823, 4.000, 4.123, 4.242, 4.359, 4.472,
        4.582, 4.690, 4.796, 4.899, 5.000, 5.099, 5.196, 5.291, 5.385, 5.477
    };
    return(root[ in - 1 ]);

}

/******************************************************************************
 *  Speed Loop KPI control;
 *  input required speed and feedback speed;
 *  output pwm duty cycle
 ******************************************************************************/
int16_t SpeedKPI(uint16_t speed_fb, uint16_t speed_req)
{
    static int32_t delta, proportion, proportionlast, integral, integralsum, kpiout;
    //  float lowpassfactor = 0.7;

    integralsum = motor_ctrl.integral_sum;
    proportionlast = motor_ctrl.proportion;
    delta = speed_req - speed_fb;

    /* TODO: low pass filter
    proportion = (1 - lowpassfactor) * delta + lowpassfactor * proportionlast;
    proportionlast = proportion;
    */

    proportion = motor_ctrl.KP * delta;
    integral = motor_ctrl.KI * delta;
    integralsum += integral;

    if (integralsum > (motor_ctrl.KID * UPPER_OUT_LIMIT_VM))
        integralsum = motor_ctrl.KID * UPPER_OUT_LIMIT_VM;
    else if (integralsum < (-motor_ctrl.KID * UPPER_OUT_LIMIT_VM))
        integralsum = -motor_ctrl.KID * UPPER_OUT_LIMIT_VM;

    motor_ctrl.integral_sum = integralsum;
    motor_ctrl.proportion = proportion;
    kpiout = (proportion / motor_ctrl.KPD) + (integralsum / motor_ctrl.KID);

    if (kpiout > UPPER_OUT_LIMIT_VM)
        kpiout = UPPER_OUT_LIMIT_VM;
    else if (kpiout < LOWER_OUT_LIMIT_VM)
        kpiout = LOWER_OUT_LIMIT_VM;

    return (( int16_t ) kpiout);
}

/*******************************************************************************
 * Set motor speed in RPM as pre-defined in file motoparam.h
 *******************************************************************************/
void MotorSetSpeed(uint16_t speed)
{
    if ((speed == TARGET_SPEED25) || (speed == TARGET_SPEED30) ||
        (speed == TARGET_SPEED35) || (speed == TARGET_SPEED40))
        motor_ctrl.speed_mech = speed;
    else
        motor_ctrl.speed_mech = TARGET_SPEED30;
}

/*******************************************************************************
 * get motor speed in hz;
 * get from CMTTIMER;
 *******************************************************************************/
uint16_t MotorGeteSpeed(void)
{
    motor_ctrl.speed_e = (SystemCoreClock >> 10) / (motor_ctrl.CMT_period * 6);
    return (motor_ctrl.speed_e);
}

/*******************************************************************************
 * get motor speed in RPM
 *******************************************************************************/
uint16_t MotorGetmSpeed(void)
{
    motor_ctrl.speed_mech = (MotorGeteSpeed() * 60) / NUM_POLE_PAIRS;
    return (motor_ctrl.speed_mech);
}

/*******************************************************************************
 * Convert motor speed in RPM to CMT period for easy using in SW
 * f = n * p/60; n=rpm, p = motor pole pairs;
 * the period is for computation phase switch CMTPERIOD;
 *******************************************************************************/
uint16_t MotorSpeed2CMTPeriod(uint16_t speed)
{
    uint32_t period;
    uint16_t f;

    f = (speed / 60) * NUM_POLE_PAIRS; // per e-round moved;
    f *= 6;                            // 6-step per move;
    period = (SystemCoreClock >> 10) / f;
    return (period);
}

/******************************************************************************
 *  Accelerate motor speed to required speed;
 * accelerate motor via reduce cmt period
 ******************************************************************************/
void SpeedAccelerate(uint16_t speed, uint16_t steps)
{
    float period_current, period_need;

    period_current = motor_ctrl.CMT_period;
    //  period_current = CMTTIMER->MOD;
    period_need = ( uint16_t ) MotorSpeed2CMTPeriod(speed);

    if (steps > 30) // TODO: maybe less.
        return;
    /*
        if ((uint16_t)period_current > (uint16_t)period_need)
            period_current *= (1 - MinSqrt(steps + 1) + MinSqrt(steps)); // TODO:
        else
            period_current += period_current * (MinSqrt(steps + 1) - MinSqrt(steps));
    */
    if (( uint16_t ) period_current > ( uint16_t ) period_need)
        period_current -= ACC * (period_current - period_need);
    else
        period_current += ACC * (period_need - period_current);

    if (( uint16_t ) (period_current - period_need) <= 2) // TODO: min 1 round will be 6*7;
        period_current = period_need;

    period_current = ( uint16_t ) period_current;
    motor_ctrl.CMT_period = period_current;
    SetCMTPeriod(motor_ctrl.CMT_period);
}

/******************************************************************************
 * 16ms loop control, 16ms@16khz PWM period;
 * Motor speed and/o current loop PI regulate
 * fault detect
 * control PWMTIMER duty cycle for Motor Speed control
 ******************************************************************************/
void MotorSpeedKPI(void)
{
    uint16_t kpiout, duty;

    //  speed feedback and adjust
    motor_ctrl.speed_fb = MotorGetmSpeed();
    kpiout = SpeedKPI(motor_ctrl.speed_fb, motor_ctrl.speed_mech);
    duty = motor_ctrl.duty_cycle + kpiout / 100; // TODO: coefficients?
    motor_ctrl.duty_cycle = duty;
    SetPWMDutyCycle(duty);
}

/******************************************************************************
 * 1ms @ PWM period @16Khz; check over current
 * Motor current loop PI regulate
 ******************************************************************************/
void MotorOverCurrent(void)
{
    // TODO:Set second ADC for iDcb
    ADC_RegularChannelConfig(ADC2, IDcbus, 1, ADC_SampleTime_239Cycles5);
    ADC_SoftwareStartConvCmd(ADC2, ENABLE);
    while (!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC));
    motor_ctrl.iDcb = ADC_GetConversionValue(ADC2);
    ADC_ClearFlag(ADC2, ADC_FLAG_EOC);
    motor_ctrl.iDcb *= motor_ctrl.Vrefd;
    motor_ctrl.iDcb *= 10; //  I = U/R sample R6=0.1 ohm

    if (motor_ctrl.iDcb >= 2.6) // NOTE maybe higher need debug to fix
    {
        flags = OVERCURRENT;
    }

}