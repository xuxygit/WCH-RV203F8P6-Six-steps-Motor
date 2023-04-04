/********************************** (C) COPYRIGHT *******************************
 * File Name          : statemachine.h
 * Author             : XUXY
 * Version            : V1.0.0
 * Date               : 2023/03/06
 * Description        : provide header files of motor state machine.
 * Copyright (c) 2023   Shenzhen NaXiang Co., Ltd.
 * Version            : V1.0.0 *
 *****************************************************************************/
#ifndef _STATEMACHINE_H_
#define _STATEMACHINE_H_


#include "ch32v20x_conf.h"

void StateMachine(void);
void MotorInit(void);
void MotorReset(void);
void MotorStop(void);
void MotorStart(void);
void MotorAlign(void);
void MotorOpenLoop(void);
void MotorRun(void);
void MotorFault(void);
uint8_t MotorZCDetecting(void);

#endif