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

extern void Error_Handler(void);






void NVIC_Init(void) {
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3); // 3 bit / 1 bit

  HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  HAL_NVIC_SetPriority(USART6_IRQn, 7, 1);
  HAL_NVIC_EnableIRQ(USART6_IRQn);


  // spi
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  HAL_NVIC_SetPriority(SPI2_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);
  /*uart
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 2, 0);   
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  */

  // led driver
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  // adc
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}










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

int handledErrorNum = -1;

void Default_Handler(void) {
  handledErrorNum = 0;
  Error_Handler();
}

void NMI_Handler(void) {
  handledErrorNum = 1;
  Error_Handler();
}

/*
void HardFault_Handler(void) {
  handledErrorNum = 2;
  Error_Handler();
}*/

void MemManage_Handler(void) {
  handledErrorNum = 3;
  Error_Handler();
}

void BusFault_Handler(void) {
  handledErrorNum = 4;
  Error_Handler();
}

void UsageFault_Handler(void) {
  handledErrorNum = 5;
  Error_Handler();
}

void SVC_Handler(void) {
  handledErrorNum = 6;
  Error_Handler();
}

void DebugMon_Handler(void) {
  handledErrorNum = 7;
  Error_Handler();
}
 
 void PendSV_Handler(void) {
  handledErrorNum = 8;
  Error_Handler();
 }

void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
/* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
volatile uint32_t r0;
volatile uint32_t r1;
volatile uint32_t r2;
volatile uint32_t r3;
volatile uint32_t r12;
volatile uint32_t lr; /* Link register. */
volatile uint32_t pc; /* Program counter. */
volatile uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    /* When the following line is hit, the variables contain the register values. */
    for( ;; );
}
/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
static void HardFault_Handler( void ) __attribute__( ( naked ) );

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
static void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
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
