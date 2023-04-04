#include "hwconfig.h"
#include "motoctrl.h"

/*
* MOSFET Channel change sequence:
* AB`->AC`->BC`->BA`->CA`->CB`ADC map in HW as:
* phase U,  V,  W output adc
    0   +   -   /         ch3
    1   +   /   -         ch2
    2   /   +   -         ch1
    3   -   +   /         ch3
    4   -   /   +         ch2
    5   /   -   +         ch1
    6   +   -   -
TIM_CCx_Enable = 0x0001
TIM_CCx_Disable = 0;
TIM_CCxN_Enable = 0x0004
TIM_CCxN_Disable = 0;
*/
const PWMCtrl_t CMTTable[ 8 ] = {
    {1,0,0,4,0,0,ADC_InjectedChannel_3}, // [0] - sector 0ADC_InjectedChannel_3
    {1,0,0,0,0,4,ADC_InjectedChannel_2}, // [1] - sector 1
    {0,0,1,0,0,4,ADC_InjectedChannel_1}, // [2] - sector 2
    {0,4,1,0,0,0,ADC_InjectedChannel_3}, // [3] - sector 3
    {0,4,0,0,1,0,ADC_InjectedChannel_2}, // [4] - sector 4
    {0,0,0,4,1,0,ADC_InjectedChannel_1}, // [5] - sector 5
    {1,0,0,4,0,4,ADC_Channel_TempSensor}, // [6] - alignment vector (combination of sectors 0 & 1)
    {0,0,0,0,0,0,ADC_Channel_Vrefint}, // [7] - PWM Off
};
Motor_Ctrl_t motor_ctrl;
uint8_t i = 0;

/**************************************************************************
LED1 = PA3, LED2 = PA2, LED3 = PA1, LED4 = PA0
**************************************************************************/
void LedInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LED1 | LED2 | LED3 | LED4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_WriteBit(GPIOA, LED1, 0);
    GPIO_WriteBit(GPIOA, LED2, 1);
    GPIO_WriteBit(GPIOA, LED3, 1);
    GPIO_WriteBit(GPIOA, LED4, 1);

}

/**************************************************************************
Key on GPIO PA6
Key on rise exti_line0  interrupt
**************************************************************************/
void KeyInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = { 0 };
    EXTI_InitTypeDef EXTI_InitStructure = { 0 };
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能复用功能时钟和GPIOA时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;                  //配置GPIO引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;              //配置GPIO上拉输入模式
    GPIO_Init(GPIOA, &GPIO_InitStructure);                     //初始化GPIOA

    /* GPIOA ----> EXTI_Line6 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6); //指定中断/事件线的输入源，实际上是设定外部中断配置寄存器AFIO_EXTICRx的值，此处为PA6
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;                   //EXTI中断/事件线选择，此处选择EXTI_Line6
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;        //EXTI模式选择，此处选择为产生中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;    //EXTI边沿触发事件，此处选择为下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                  //使能EXTI线
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;         //使能EXTI中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //设置抢占优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;         //设置子优先级为2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);                            //中断优先级分组初始化
}

/**************************************************************************
*   Using Timer2 for CMT
*
**************************************************************************/
void CMTTimerInit(uint16_t CMT_Period)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = { 0 };
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //使能TIM2时钟

    TIM_TimeBaseStructure.TIM_Period = CMT_Period - 1;   //指定下次更新事件时要加载到活动自动重新加载寄存器中的周期值。
    TIM_TimeBaseStructure.TIM_Prescaler = 1024 - 1; //指定用于划分TIM时钟的预分频器值。
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频因子
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM计数模式，向上计数模式
    TIM_TimeBaseInit(CMTTIMER, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位

    TIM_ITConfig(CMTTIMER, TIM_IT_Update, ENABLE); //使能TIM2中断，允许更新中断

    //初始化TIM NVIC，设置中断优先级分组
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;           //TIM2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //设置抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //设置响应优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //使能通道1中断
    NVIC_Init(&NVIC_InitStructure); //初始化NVIC

    TIM_ARRPreloadConfig(CMTTIMER, ENABLE);
    TIM_Cmd(CMTTIMER, ENABLE);
    
}


/**************************************************************************
*   Using Time1 for generator PWM drive phase
*   Description : Initializes TIM1 complementary output and dead time.
*               CH1,CHN1,CH2,CHN2,CH3,CHN3 as MOSFET driver
*               CH4 no no output pin, using as ADC trigger;
**************************************************************************/
void PWMTimerInit(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure = { 0 };
    TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = { 0 };
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure = { 0 };

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_TIM1, ENABLE);

    /* configure PA8/PA9/PA10 TIM1_CH1,2,3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* configure PB13/PB14/PB15 TIM1_CH1N,2N,3N */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //时基结构体初始化配置
    TIM_TimeBaseInitStructure.TIM_Period = PWMTIMER_Period - 1;  //设置重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = PWMTIMER_Prescaler - 1;  //设置预分频器
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //设置分频因子
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  //设置向上计数模式
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = TRUE;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);  //初始化定时器

    //CH1~CH3 输出比较结构体初始化
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //配置为PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;  //互补输出使能
    TIM_OCInitStructure.TIM_Pulse = PWMTIMER_Period / 2;  //设置占空比大小
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //输出通道电平极性配置
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;  //互补输出通道电平极性配置
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;  //输出通道空闲电平极性配置
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;  //互补输出通道空闲电平极性配置

    TIM_OC1Init(PWMTIMER, &TIM_OCInitStructure);  //初始化
    TIM_OC2Init(PWMTIMER, &TIM_OCInitStructure);
    TIM_OC3Init(PWMTIMER, &TIM_OCInitStructure);

    //  CH4 configuration 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //配置为PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;  //输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;  //互补输出使能
    TIM_OCInitStructure.TIM_Pulse = PWMTIMER_Period / 2 + 8;  // 8 more for demag;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC4Init(PWMTIMER, &TIM_OCInitStructure);

    //刹车和死区结构体初始化，有关刹车和死区结构体的成员具体可参考刹车和死区寄存器（TIMx_BDTR）的描述
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;

    //输出比较信号死区时间配置，具体如何计算可参考 BDTR:DTG[7:0]的描述
    //这里配置的死区时间为152ns
    TIM_BDTRInitStructure.TIM_DeadTime = PWM_DEAD_TIME;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;

    //当BKIN引脚检测到高电平的时候，输出比较信号被禁止，就好像是刹车一样
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(PWMTIMER, &TIM_BDTRInitStructure);


    TIM_OC1PreloadConfig(PWMTIMER, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(PWMTIMER, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(PWMTIMER, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(PWMTIMER, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(PWMTIMER, ENABLE);

    TIM_Cmd(PWMTIMER, ENABLE);
    TIM_CtrlPWMOutputs(PWMTIMER, ENABLE);  //开启定时器PWM输出


}



/*******************************************************************************
* Function Name  : ADCInit
* Description    : Initializes ADC collection.
* in our system  : using PA4,5,7 as ADC1 channel 4,5,7 while triggle by TIM1
*                  working in injection mode,
*                  Using PB0 ADC2 channel 8 for iDcbus AD, working in regule mode;
* Input          : None
* Return         : None
*******************************************************************************/
void ADCInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef  ADC_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    int16_t Calibrattion_Val1;


    /* GPIOB PB0 ADC8 config*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ADC2 trigger config */
    ADC_DeInit(ADC2);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC2, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC2, IDcbus, 1, ADC_SampleTime_239Cycles5); // iDcbus as regular adc
    ADC_RegularChannelConfig(ADC2, ADC_Channel_Vrefint, 1, ADC_SampleTime_239Cycles5);
    ADC_Cmd(ADC2, ENABLE);
    ADC_BufferCmd(ADC2, DISABLE); //disable buffer
    ADC_ResetCalibration(ADC2);
    while (ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while (ADC_GetCalibrationStatus(ADC2));
    Calibrattion_Val1 = Get_CalibrationValue(ADC2);
    ADC_BufferCmd(ADC2, ENABLE); //enable buffer

    /* GPIOA PA4,5,7 as ADC1 channel 4,5,7 config*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ADC1 trigger config */
    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_AlterTrig;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigInjecConv_T1_CC4;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 3;
    ADC_InitStructure.ADC_OutputBuffer = ADC_OutputBuffer_Disable;
    ADC_InitStructure.ADC_Pga = ADC_Pga_1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_InjectedSequencerLengthConfig(ADC1, 3);
    //    ADC_InjectedChannelConfig(ADC1, BEMF_U, 1, ADC_SampleTime_28Cycles5);
    //    ADC_InjectedChannelConfig(ADC1, BEMF_V, 1, ADC_SampleTime_28Cycles5);
    ADC_InjectedChannelConfig(ADC1, BEMF_W, 1, ADC_SampleTime_28Cycles5);
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);
    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // rising interrupt after conversion done;
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_BufferCmd(ADC1, DISABLE); //disable buffer
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
    Calibrattion_Val1 = Get_CalibrationValue(ADC1);

    ADC_BufferCmd(ADC1, ENABLE); //enable buffer
}

/*******************************************************************************
* Function Name  : GetADCVal
* Description    : Returns ADCx conversion result data.
* Return         : val: The Data conversion value.
*******************************************************************************/
/*
uint16_t GetADCVal(uint8_t ch)
{
    uint16_t val;

    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    val = ADC_GetConversionValue(ADC1);
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    return val;
}
*/

/*************************************************************************
 * Function Name: SetDutyCycle
 * Parameters: duty cycle value  is number of percent, 40 -> 40%;
* duty cycle:
    duty  = (On time ) / Period = ChxVal / CAR_value;
 * Return: none
 * Description: Set PWM duty cycle of PWMTIMER change phase voltage
 *************************************************************************/
void SetPWMDutyCycle(uint8_t duty)
{
    if (duty > 90)
        duty = 90;

    // TIM_SetCompare1(TIM1, (uint32_t) (duty * PWMTIMER_Period) / 100);
    // TIM_SetCompare2(TIM1, (uint32_t) (duty * PWMTIMER_Period) / 100);
    // TIM_SetCompare3(TIM1, (uint32_t) (duty * PWMTIMER_Period) / 100);
    PWMTIMER->CH1CVR = ( uint32_t ) (duty * PWMTIMER_Period) / 100;
    PWMTIMER->CH2CVR = ( uint32_t ) (duty * PWMTIMER_Period) / 100;
    PWMTIMER->CH3CVR = ( uint32_t ) (duty * PWMTIMER_Period) / 100;
    TIM_ARRPreloadConfig(PWMTIMER, ENABLE);
}

/*************************************************************************
 * Function Name: SetPwmOutput
 * Parameters: sector number
 * Return: none
 * Description: set PWM output configuration based on selected sector
 *************************************************************************/
void SetPWMPhase(uint8_t phase)
{
    /*
        uint16_t temp;
        phase = sector;

        temp = TIM1->CCER;
        temp &= ~(TIM_CC1E | TIM_CC1NE | TIM_CC2E | TIM_CC2NE | TIM_CC3E | TIM_CC3NE);
        temp |= CMTTable[ phase ].Channel_1 | CMTTable[ phase ].Channel_1N | \
            CMTTable[ phase ].Channel_2 | CMTTable[ phase ].Channel_2N | \
            CMTTable[ phase ].Channel_3 | CMTTable[ phase ].Channel_3N;
        TIM1->CCER = temp;
    */
    TIM_CCxCmd(TIM1, TIM_Channel_1, CMTTable[ phase ].Channel_1);
    TIM_CCxNCmd(TIM1, TIM_Channel_1, CMTTable[ phase ].Channel_1N);

    TIM_CCxCmd(TIM1, TIM_Channel_2, CMTTable[ phase ].Channel_2);
    TIM_CCxNCmd(TIM1, TIM_Channel_2, CMTTable[ phase ].Channel_2N);

    TIM_CCxCmd(TIM1, TIM_Channel_3, CMTTable[ phase ].Channel_3);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, CMTTable[ phase ].Channel_3N);
}

/*************************************************************************
 * Function Name: Sector from  0 to 5 in running state;
 * Parameters: sector number
 *************************************************************************/
uint8_t NextSector(uint8_t sector)
{
    sector++;
    sector %= 6;
    motor_ctrl.phase_sector = sector;

    return (sector);
}

/*************************************************************************
 * Function Name: Set CMTTIMER period;
 * Parameters: period
 *************************************************************************/
void SetCMTPeriod(uint16_t period)
{
    CMTTIMER->CNT = 0;
    CMTTIMER->ATRLR = period;
    // TIM_SetAutoreload(CMTTIMER, period);
    // TIM_SetCounter(CMTTIMER, 0);
}

/*************************************************************************
 * Function Name: Toggle select GPIO Pin;
 * Parameters: Pin number
 *************************************************************************/
void GPIO_PinToggle(uint32_t pin)
{
    GPIO_WriteBit(GPIOA, pin, (i == 0) ? (i = Bit_SET) : (i = Bit_RESET));
}
