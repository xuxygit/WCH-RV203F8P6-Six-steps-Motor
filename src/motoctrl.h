/********************************** (C) COPYRIGHT *******************************
 * File Name          : hwconfig.h
 * Author             : XUXY
 * Version            : V1.0.0
 * Date               : 2023/03/06
 * Description        : Motor hardware control head file.
 * provide header files of motor control parameters.
 * Copyright (c) 2023   Shenzhen NaXiang Co., Ltd.
 * Version            : V1.0.0 *
 *****************************************************************************/
#ifndef _MOTOCTRL_H_
#define _MOTOCTRL_H_
#include "ch32v20x_conf.h"


typedef enum
{
    INIT,	   /* 0 */
    STOP,	   /* 1 */
    START,	   /* 2 */
    RUN,	   /* 3 */
    ALIGNMENT, /* 4 */
    OPENLOOP,  /* 5 */
    SPIN,	   /* 6 */
    FAULT	   /* 7 */
} SysStatus_t;

typedef enum
{
    NONE,
    ALIGN_OK,
    OPENLOOP_OK,
    OPENLOOP_FAIL,
    ZCDETECT_OK,
    ZCDETECT_FAIL,
    CLOSELOOPRUN_READY,
    CLOSELOOPRUN_OK,
    CLOSELOOPRUN_FAIL,
    OVERCURRENT
} Flags_t;

typedef struct
{
    /* HW config parameters*/
    uint16_t PWM_period;  // period of MOSFET PWM deriver, fixed in 2500 @16khz
    uint16_t CMT_period;  // period of MOSFET phase change, control CMTTIMER
    uint8_t phase_sector; // MOSFET phase sector index,0-5;
    uint8_t duty_cycle;	  // dutycycle of MOSFET PWM deriver,control PWMTIMER

    /* Timing and counter para */
    uint32_t PWM_step_count; // count of PWM switch, @16khz per count;
    uint16_t CMT_step_count; // CMT phase change count;
    uint16_t round_count;	 // motor run rounds count;
    uint16_t demag_time;	 // demag time for ADC,~25% of step period

    /* Zero crossing values*/
    uint16_t ZC_count;	   // measure the time between two zero crossing point;
    uint16_t ZC_time[ 6 ];   // ZC detected event steps and time in CMT->CN;
    uint16_t ZC_lead_time; // Zero crossing point lead 50% of CMT switch;
    uint16_t Bemf_volt[ 3 ]; //	6 as each PWM E-round(6-step) got 6 zc point;
    uint16_t Bemf_volt_index;
    uint16_t Bemf_volt_nopow;
    uint16_t Bemf_volt_pow;
    uint16_t Bemf_sum;
    uint8_t adc_channel;
    float iDcb;
    uint16_t Vrefd;

    /* speed PID control para*/
    uint32_t accel; /*!< Acceleration start-up parameter*/
    uint16_t KP;	/*!< KP parameter for PI regulator */
    uint16_t KI;	/*!< KI parameter for PI regulator */
    uint16_t KPD;
    uint16_t KID;
    uint16_t proportion;
    uint32_t integral_sum;
    uint32_t speed_fb;	 // estimate speed
    uint16_t speed_e;	 // speed of motor at e
    uint16_t speed_mech; // motor speed in rpm;

    /* uart debug set  */
    uint32_t uart_set_cmd;
    uint32_t uart_set_value;

} Motor_Ctrl_t;

/*
 * Channel_1, Channel_1N,Channel_2, Channel_2N,Channel_3, Channel_3N,
 * BEMF ADC channel
 *
*/
typedef struct
{
    uint16_t Channel_1;
    uint16_t Channel_1N;
    uint16_t Channel_2;
    uint16_t Channel_2N;
    uint16_t Channel_3;
    uint16_t Channel_3N;
    uint8_t ADC_Channel;
}PWMCtrl_t;

typedef enum
{
    FALSE = 0,
    TRUE = !FALSE
}bool;

float MinSqrt(uint16_t);
int16_t SpeedKPI(uint16_t, uint16_t);
void SpeedAccelerate(uint16_t, uint16_t);
void MotorSetSpeed(uint16_t);
uint16_t MotorGeteSpeed(void);
uint16_t MotorGetmSpeed(void);
uint16_t MotorSpeed2CMTPeriod(uint16_t);
void MotorSpeedKPI(void);
void MotorOverCurrent(void);

#endif