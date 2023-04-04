/********************************** (C) COPYRIGHT *******************************
 * File Name          : motorparam.h
 * Author             : XUXY
 * Version            : V1.0.0
 * Date               : 2023/03/06
 * Description        : provide header files of motor parameters.
 * Copyright (c) 2023   Shenzhen NaXiang Co., Ltd.
 * Version            : V1.0.0 *
 *****************************************************************************/
#ifndef _MOTOPARAM_H_
#define _MOTOPARAM_H_

 /* ****************************************************************************
                       ###### BASIC PARAMETERS ######
  **************************************************************************** */
#define NUM_POLE_PAIRS 7    /*!< Number of Motor Pole pairs */
#define TARGET_SPEED25 2500 /*!< Target speed in closed loop control */
#define TARGET_SPEED30 3000
#define TARGET_SPEED35 3500
#define TARGET_SPEED40 4000
#define CMT_PERIOD 		500

  /* ****************************************************************************
                         ###### ADVANCED PARAMETERS ######
   **************************************************************************** */
   /*!< ********************* Open loop control *********************************/
#define STARTUP_CURRENT_REFERENCE 1000 /*!< StartUP Current Reference */
#define ACC 0.150                      	/*!< Mechanical acceleration rate */
#define OPENLOOP_SPEED 500             /* ~15% of normal speed*/
#define MINIMUM_ACC 50                 /*!< Mechanical acceleration rate for BIG load application */
#define NUMBER_OF_STEPS 20000          /*!< Number of elements for motor start-UP (max value 65535)*/
#define TIME_FOR_ALIGN 100             /*!< Time for alignment (msec)*/
#define BUTTON_DELAY 1000              /*!< Delay time to enable push button for new command (1 = 1msec)*/
#define NUMBER_ZCR 12                  /*!< Number of zero crossing event during the startup for closed loop control begin */
#define DUTY_CYCLE_ALIGN 70
#define DUTY_CYCLE_OP 65
/*!< ********************* Closed Loop control - VOLTAGE MODE *******************/
#define KP_GAIN_VM 100           /*!< Kp parameter for PI regulator */
#define KI_GAIN_VM 30            /*!< Ki parameter for PI regulator */
#define KP_DIV_VM 8192           /*!< Kp parameter divider for PI regulator */
#define KI_DIV_VM 8192           /*!< Ki parameter divider for PI regulator */
#define LOWER_OUT_LIMIT_VM 625   /*!< Low Out value of PI regulator */
#define UPPER_OUT_LIMIT_VM 1000  /*!< High Out value of PI regulator */
#define DUTY_CYCLE_INIT_VALUE 65 /*!< Initial duty cycle value during startup */
#define KPI_DUTYCYCLE_RATIO 0.7  /*TODO, not know yet, should test*/
/*!< Zero Crossing parameters */
#define BEMF_THRESHOLD 80 /*!< Zero Crossing threshold */

/*!< Speed filtering parameters */
#define FILTER_DEEP 20 /*!< Number of bits for digital filter */
#define HFBUFFERSIZE 10
#define ADC_SPEED_TH 82 /*!<Fixed threshold to change the target speed (t.b.f) */

/* Over current limit  */
#define OVERCURRENTLIMIT 250 // 2A on 0.1 ohm sensor @3.3v 12bit;
#endif
