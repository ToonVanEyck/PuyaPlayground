/**
 ******************************************************************************
 * @file    main.c
 * @author  MCU Application Team
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
 * All rights reserved.</center></h2>
 *
 * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx_bsp_printf.h"
#include "py32f0xx_hal.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void APP_ErrorHandler(void);

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef sConfig;
uint32_t aADCxConvertedData[6];
TIM_HandleTypeDef TimHandle;
TIM_MasterConfigTypeDef sMasterConfig;
TIM_OC_InitTypeDef sConfigOC;
UART_HandleTypeDef UartHandle;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_LedConfig(void);
static void APP_TimerInit(void);
static void APP_AdcConfig(void);
static void APP_DmaInit(void);
static void APP_UartInit(void);

int main(void)
{
    HAL_Init();

    BSP_USART_Config();

    APP_LedConfig();

    APP_TimerInit();

    APP_AdcConfig();

    APP_DmaInit();

    // APP_UartInit();

    printf("Started Puya App\r\n");
    while (1) {
        // uint8_t buffer[4];
        // HAL_UART_Receive(&UartHandle, buffer, sizeof(buffer), HAL_MAX_DELAY);
        // HAL_UART_Transmit(&UartHandle, buffer, sizeof(buffer), HAL_MAX_DELAY);
    }
}

static void APP_LedConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void APP_AdcConfig(void)
{
    __HAL_RCC_ADC_FORCE_RESET();
    __HAL_RCC_ADC_RELEASE_RESET(); /* Reset ADC */
    __HAL_RCC_ADC_CLK_ENABLE();    /* Enable ADC clock */

    AdcHandle.Instance = ADC1;
    if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK) {
        APP_ErrorHandler();
    }
    AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1; /* ADC clock no division */
    AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;           /* 12bit */
    AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;           /* Right alignment */
    AdcHandle.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD; /* Backward */
    AdcHandle.Init.EOCSelection = ADC_EOC_SEQ_CONV;           /* End flag */
    AdcHandle.Init.LowPowerAutoWait = ENABLE;
    AdcHandle.Init.ContinuousConvMode = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.ExternalTrigConv = ADC1_2_EXTERNALTRIG_T1_TRGO;                /* External trigger: TIM1_TRGO */
    AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING; /* Triggered by both edges */
    AdcHandle.Init.DMAContinuousRequests = ENABLE;                                /* No DMA */
    AdcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    AdcHandle.Init.SamplingTimeCommon = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
        APP_ErrorHandler();
    }

    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.Channel = ADC_CHANNEL_0;
    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
        APP_ErrorHandler();
    }
    sConfig.Channel = ADC_CHANNEL_1;
    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
        APP_ErrorHandler();
    }
    sConfig.Channel = ADC_CHANNEL_2;
    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
        APP_ErrorHandler();
    }
    sConfig.Channel = ADC_CHANNEL_3;
    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
        APP_ErrorHandler();
    }
    sConfig.Channel = ADC_CHANNEL_4;
    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
        APP_ErrorHandler();
    }
    sConfig.Channel = ADC_CHANNEL_5;
    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
        APP_ErrorHandler();
    }
    /* Start ADC with DMA */
    if (HAL_ADC_Start_DMA(&AdcHandle, aADCxConvertedData, 6) != HAL_OK) {
        APP_ErrorHandler();
    }
}

static void APP_TimerInit(void)
{
    // (800 * 1000) / 8Mhz = 100ms
    __HAL_RCC_TIM1_CLK_ENABLE();
    TimHandle.Instance = TIM1;
    TimHandle.Init.Period = 1000 - 1;
    TimHandle.Init.Prescaler = 800 - 1;
    TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimHandle.Init.RepetitionCounter = 0;
    TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK) {
        APP_ErrorHandler();
    }
    if (HAL_TIM_OC_Init(&TimHandle) != HAL_OK) {
        APP_ErrorHandler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig);
    if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK) {
        APP_ErrorHandler();
    }

    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
    sConfigOC.Pulse = 5000;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_OC_ConfigChannel(&TimHandle, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        APP_ErrorHandler();
    }

    if (HAL_TIM_OC_Start_IT(&TimHandle, TIM_CHANNEL_4) != HAL_OK) {
        APP_ErrorHandler();
    }
}

static void APP_DmaInit(void)
{
    // __HAL_RCC_DMA_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void APP_UartInit(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_USART2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Alternate = GPIO_AF9_USART2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    UartHandle.Instance = USART2;
    UartHandle.Init.BaudRate = 115200;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits = UART_STOPBITS_1;
    UartHandle.Init.Parity = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode = UART_MODE_TX_RX;

    if (HAL_UART_Init(&UartHandle) != HAL_OK) {
        APP_ErrorHandler();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // aADCxConvertedData = hadc->Instance->DR;
    printf("ADC: %04ld  %04ld  %04ld  %04ld  %04ld  %04ld\r\n", aADCxConvertedData[0], aADCxConvertedData[1],
           aADCxConvertedData[2], aADCxConvertedData[3], aADCxConvertedData[4], aADCxConvertedData[5]);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
}

// void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
//         HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
//     }
// }

void APP_ErrorHandler(void)
{
    while (1)
        ;
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Export assert error source and line number
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    while (1) {
    }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
