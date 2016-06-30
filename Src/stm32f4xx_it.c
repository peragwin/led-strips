/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_timx_gpio_low;
extern DMA_HandleTypeDef hdma_timx_gpio_data;
extern DMA_HandleTypeDef hdma_timx_gpio_high;
extern TIM_HandleTypeDef htimx;
extern TIM_HandleTypeDef htim_dead;
extern TIM_HandleTypeDef htimFrame;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;


/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA2 stream1 global interrupt.
*/
void DMA2_Stream1_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_timx_gpio_data);

}

/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_timx_gpio_low);

}

/**
* @brief This function handles DMA2 stream5 global interrupt.
*/
void DMA2_Stream5_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_timx_gpio_high);

}

void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htimx);
}



void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}
void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart6);
}



void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim_dead);
}

/**
  * @brief  This function handles DMA Rx interrupt request.  
  * @param  None
  * @retval None    
  */
  // SPI RX
void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hspi2.hdmarx);
}

/**
  * @brief  This function handles DMA Tx interrupt request.
  * @param  None
  * @retval None  
  */
  // SPI TX
void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hspi2.hdmatx);
}

/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi2);
}

void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void ADC_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc1);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
