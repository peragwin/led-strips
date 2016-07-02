/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htimx;
TIM_HandleTypeDef htim_dead;
TIM_HandleTypeDef htimFrame;
DMA_HandleTypeDef hdma_timx_gpio_low;
DMA_HandleTypeDef hdma_timx_gpio_high;
DMA_HandleTypeDef hdma_timx_gpio_data;

/* TIM1 init function */
void TIM_Leds_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  uint32_t uwPrescalerValue = 0; //(SystemCoreClock / DATA_TIM_FREQ) - 1;
  uint32_t uhPeriodValue = DATA_TIM_PERIOD - 1;

  htimx.Instance = TIM1;
  htimx.Init.Period = uhPeriodValue;
  htimx.Init.Prescaler = uwPrescalerValue;
  htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htimx) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htimx, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htimx) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htimx, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htimx, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 1 is short (400ns) pulse
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = DATA_TIM_PULSE1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  // Channel 2 is long (800ns) pulse
  sConfigOC.Pulse = DATA_TIM_PULSE2;
  if (HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  //HAL_NVIC_SetPriority(TIM1_IRQn, 0, 2);
  //HAL_NVIC_EnableIRQ(TIM1_IRQn);


  HAL_TIM_MspPostInit(&htimx);

}

// used as a one-shot timer to assert dead period
void TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim_dead.Instance = TIM2;
  htim_dead.Init.Prescaler = 0;
  htim_dead.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_dead.Init.Period = WS2812_DEADPERIOD;
  htim_dead.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim_dead) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim_dead, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }


  HAL_TIM_MspPostInit(&htim_dead);
}






volatile uint8_t timDeadCycleCount = 0;
extern volatile uint8_t WS2812_TransferComplete;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* tim_baseHandle)
{
  // callback handles UPDATE interrupts for TIM

  if(tim_baseHandle->Instance==TIM5)
  {
    
    // // make sure to wait 50us after any transfer completes
    // if (timDeadCycleCount < (uint8_t)WS2812_DEADPERIOD) timDeadCycleCount++;
    // else
    // {
    //   timDeadCycleCount = 0;
    //   __HAL_TIM_DISABLE(tim_baseHandle);
    //   __HAL_TIM_DISABLE_IT(tim_baseHandle, TIM_IT_UPDATE);
    //   WS2812_TransferComplete = 1;

    // }

    //           GPIOC->ODR = 0xFFFF;
  }

  else if (tim_baseHandle->Instance==TIM2)
  {
    __HAL_TIM_DISABLE(&htim_dead);
    WS2812_TransferComplete = 1;
  }


}

// void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//   {
//     GPIOC->ODR = 0x0000;
//   }
// }

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {

    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* Peripheral DMA init*/

    // this stream is for copying 0xFF (output high) on TIM update
    hdma_timx_gpio_high.Instance =                  DMA2_Stream5;
    hdma_timx_gpio_high.Init.Channel =              DMA_CHANNEL_6;
    hdma_timx_gpio_high.Init.Direction =            DMA_MEMORY_TO_PERIPH;
    hdma_timx_gpio_high.Init.PeriphInc =            DMA_PINC_DISABLE;
    hdma_timx_gpio_high.Init.MemInc =               DMA_MINC_DISABLE;
    hdma_timx_gpio_high.Init.PeriphDataAlignment =  DMA_PDATAALIGN_BYTE;
    hdma_timx_gpio_high.Init.MemDataAlignment =     DMA_MDATAALIGN_WORD;
    hdma_timx_gpio_high.Init.Mode =                 DMA_NORMAL;
    hdma_timx_gpio_high.Init.Priority =             DMA_PRIORITY_VERY_HIGH;
    hdma_timx_gpio_high.Init.FIFOMode =             DMA_FIFOMODE_ENABLE;
    hdma_timx_gpio_high.Init.FIFOThreshold =        DMA_FIFO_THRESHOLD_FULL;
    hdma_timx_gpio_high.Init.MemBurst =             DMA_MBURST_INC4;
    hdma_timx_gpio_high.Init.PeriphBurst =          DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(&hdma_timx_gpio_high) != HAL_OK) Error_Handler();

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_UPDATE],hdma_timx_gpio_high); //think this is on rising edge

  
    // this stream is for copying data (output data) on TIM CC1
    hdma_timx_gpio_data.Instance =                  DMA2_Stream1;
    hdma_timx_gpio_data.Init.Channel =              DMA_CHANNEL_6;
    hdma_timx_gpio_data.Init.Direction =            DMA_MEMORY_TO_PERIPH;
    hdma_timx_gpio_data.Init.PeriphInc =            DMA_PINC_DISABLE;
    hdma_timx_gpio_data.Init.MemInc =               DMA_MINC_ENABLE;    // this one should increment pointer to the buffer
    hdma_timx_gpio_data.Init.PeriphDataAlignment =  DMA_PDATAALIGN_BYTE;
    hdma_timx_gpio_data.Init.MemDataAlignment =     DMA_MDATAALIGN_WORD;
    hdma_timx_gpio_data.Init.Mode =                 DMA_NORMAL;
    hdma_timx_gpio_data.Init.Priority =             DMA_PRIORITY_VERY_HIGH;
    hdma_timx_gpio_data.Init.FIFOMode =             DMA_FIFOMODE_ENABLE;
    hdma_timx_gpio_data.Init.FIFOThreshold =        DMA_FIFO_THRESHOLD_FULL;
    hdma_timx_gpio_data.Init.MemBurst =             DMA_MBURST_INC4;
    hdma_timx_gpio_data.Init.PeriphBurst =          DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(&hdma_timx_gpio_data) != HAL_OK) Error_Handler();

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC1],hdma_timx_gpio_data);  //falling edge?


    // this stream is for copying 0x00 (output low) on TIM CC3
    // also has the callback for when dma streaming is complete
    hdma_timx_gpio_low.Instance =                   DMA2_Stream2;
    hdma_timx_gpio_low.Init.Channel =               DMA_CHANNEL_6;
    hdma_timx_gpio_low.Init.Direction =             DMA_MEMORY_TO_PERIPH;
    hdma_timx_gpio_low.Init.PeriphInc =             DMA_PINC_DISABLE;
    hdma_timx_gpio_low.Init.MemInc =                DMA_MINC_DISABLE;
    hdma_timx_gpio_low.Init.PeriphDataAlignment =   DMA_PDATAALIGN_BYTE;
    hdma_timx_gpio_low.Init.MemDataAlignment =      DMA_MDATAALIGN_BYTE;
    hdma_timx_gpio_low.Init.Mode =                  DMA_NORMAL;
    hdma_timx_gpio_low.Init.Priority =              DMA_PRIORITY_MEDIUM;
    hdma_timx_gpio_low.Init.FIFOMode =              DMA_FIFOMODE_ENABLE;
    hdma_timx_gpio_low.Init.FIFOThreshold =         DMA_FIFO_THRESHOLD_FULL;
    hdma_timx_gpio_low.Init.MemBurst =              DMA_MBURST_INC4;
    hdma_timx_gpio_low.Init.PeriphBurst =           DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(&hdma_timx_gpio_low) != HAL_OK) Error_Handler();

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC2],hdma_timx_gpio_low);     //think this is on falling edge
    __HAL_DMA_ENABLE_IT(&hdma_timx_gpio_low, DMA_IT_TC);                       //flag it when transfer complete
    


    // register callbacks

    if (HAL_DMA_RegisterCallback(&hdma_timx_gpio_data,
                                 HAL_DMA_XFER_CPLT_CB_ID,
                                 &FrameXferCompleteCallback)
        != HAL_OK) Error_Handler();
    
    if (HAL_DMA_RegisterCallback(&hdma_timx_gpio_data,
                                 HAL_DMA_XFER_ERROR_CB_ID,
                                 &FrameXferErrorCallback)
        != HAL_OK) Error_Handler();

  }



  else if (tim_baseHandle->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();


    HAL_NVIC_SetPriority(TIM2_IRQn, 3, 2);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
}

extern void FrameXferCompleteCallback(DMA_HandleTypeDef *hdma);

void FrameXferErrorCallback(DMA_HandleTypeDef *hdma)
{
  Error_Handler();
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(timHandle->Instance==TIM1)
  {

    /**TIM1 GPIO Configuration    
    PA8     ------> TIM2_CH1
    PA9     ------> TIM2_CH2 
    
    GPIO_InitStruct.Pin =       GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode =      GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull =      GPIO_NOPULL;
    GPIO_InitStruct.Speed =     GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    */

  // make sure to actually enable PWM!!
  TIM_CCxChannelCmd(timHandle->Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  TIM_CCxChannelCmd(timHandle->Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);

  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_UPDATE]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC2]);
  }

  else if(tim_baseHandle->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_DISABLE();

    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
