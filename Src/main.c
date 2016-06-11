/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#define PI 3.1415926
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// buffer size is num_leds * 24 bits/led
// 60 * 24 = 1440
#define LEDS_PER_STRIP 60
#define BUFFSIZE 1440
  //0

// put these in memory so they can be copied via DMA
volatile uint8_t DMA_IO_FrameBuffer[BUFFSIZE];
volatile uint32_t DMA_IO_High = 0xFFFFFFFFU;
volatile uint32_t DMA_IO_Low  = 0x00000000U;

volatile uint8_t WS2812_TransferComplete = 1;

extern TIM_HandleTypeDef htimx;
extern DMA_HandleTypeDef hdma_timx_gpio_low;     // DMA1_stream4 sets data gpios low
extern DMA_HandleTypeDef hdma_timx_gpio_data;    // DMA1_stream5 sets data gpios to buffer value
extern DMA_HandleTypeDef hdma_timx_gpio_high;    // DMA1_stream6 sets data gpios high


typedef struct
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} ColorTypeDef;

#include "colors.out"

const ColorTypeDef CBlack      = {0x00, 0x00, 0x00};
const ColorTypeDef CRed        = {0xFF, 0x00, 0x00};
const ColorTypeDef CGreen      = {0x00, 0xFF, 0x00};
const ColorTypeDef CBlue       = {0x00, 0x00, 0xFF};
const ColorTypeDef CCyan       = {0x00, 0xFF, 0xFF};
const ColorTypeDef CMagenta    = {0xFF, 0x00, 0xFF};
const ColorTypeDef CYellow     = {0xFF, 0xFF, 0x00};
const ColorTypeDef CWhite      = {0xFF, 0xFF, 0xFF};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void DMA_IO_SendBuffer(uint8_t* frameData, uint16_t buffsize);

void Debug_Setup(void);

void DebugGood_Handler(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


void DMA_IO_SendBuffer(uint8_t* frameBuffer, uint16_t buffsize)
{
  while(!WS2812_TransferComplete) HAL_Delay(1);
  WS2812_TransferComplete = 0;

  // Enable DMA
  if (HAL_DMA_Start_IT(&hdma_timx_gpio_high,
                       (uint32_t)&DMA_IO_High,
                       (uint32_t)&(GPIOC->ODR),
                       buffsize)
      != HAL_OK) Error_Handler();

  if (HAL_DMA_Start_IT(&hdma_timx_gpio_data,
                       (uint32_t)&DMA_IO_FrameBuffer,
                       (uint32_t)&(GPIOC->ODR),
                       buffsize)
      != HAL_OK) Error_Handler();
  
  if (HAL_DMA_Start_IT(&hdma_timx_gpio_low,
                       (uint32_t)&DMA_IO_Low,
                       (uint32_t)&(GPIOC->ODR),
                       buffsize)
     != HAL_OK) Error_Handler();



  __HAL_TIM_ENABLE_DMA(&htimx, TIM_DMA_UPDATE);
  __HAL_TIM_ENABLE_DMA(&htimx, TIM_DMA_CC1);
  __HAL_TIM_ENABLE_DMA(&htimx, TIM_DMA_CC2);

  __HAL_TIM_CLEAR_FLAG(&htimx, (uint16_t)0xFFFFU); // clear all
  // Start counter at max value so UPDATE IT is generated immediately
  __HAL_TIM_SET_COUNTER(&htimx, (uint16_t)(DATA_TIM_PERIOD - 1));

  // Enable the timer to kick things off
  __HAL_TIM_ENABLE(&htimx);
}

void FrameBuffer_SetPixel(uint8_t* frameBuffer, uint8_t channel, uint16_t index, ColorTypeDef c)
{
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    // clear
    frameBuffer[(index*24)+i]    &= ~(0x01<<channel);
    frameBuffer[(index*24)+8+i]  &= ~(0x01<<channel);
    frameBuffer[(index*24)+16+i] &= ~(0x01<<channel);

    // set
    frameBuffer[(index*24)+i]    |= ((c.green<<i) & 0x80) >> (7-channel);
    frameBuffer[(index*24)+8+i]  |= (  (c.red<<i) & 0x80) >> (7-channel);
    frameBuffer[(index*24)+16+i] |= ( (c.blue<<i) & 0x80) >> (7-channel);
  }
}

GPIO_TypeDef *GpioC;

void Debug_Setup(void)
{
  GpioC = (GPIO_TypeDef*)GPIOC;

  // enable usasge error handler status reg
  *((uint32_t*)0xe000ed24) = 0x70000;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();

  Debug_Setup();

  /* USER CODE BEGIN 2 */

  for (uint8_t i = 0; i < BUFFSIZE/24; i++)
  {
    FrameBuffer_SetPixel(&DMA_IO_FrameBuffer, 0, i, CWhite);
  }

  uint16_t size = BUFFSIZE;
  while(0);
  DMA_IO_SendBuffer(&DMA_IO_FrameBuffer, size);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    while(!WS2812_TransferComplete)
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(500);
    }
    DebugGood_Handler();
    //DMA_IO_SendBuffer(&DMA_IO_FrameBuffer, BUFFSIZE);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    
    /* Insert delay 100 ms */
    HAL_Delay(100);
  }
  /* USER CODE END Error_Handler */ 
}

void DebugGood_Handler(void)
{
  // go for cycle period of 10 sec, or 300 frames
  const float w = PI/300;
  //uint8_t rval, gval, bval; 
  ColorTypeDef c;

  int i, d = 0;
  while(1)
  {

    if (!(d%30) || d%30==6) HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    HAL_Delay(33); // try for 30 fps
    
    /*
    c.red   = (uint8_t)(128.0f + 120.0f*sin(w*d));
    c.green = (uint8_t)(128.0f + 120.0f*sin(w*(d+200)));
    c.blue  = (uint8_t)(128.0f + 120.0f*sin(w*(d+400)));
    */

    c = ColorTable[d%COLOR_TABLE_LEN];

    for(i = 0; i < BUFFSIZE/24; i++)
    {
      FrameBuffer_SetPixel(&DMA_IO_FrameBuffer, 0, i, c);
      FrameBuffer_SetPixel(&DMA_IO_FrameBuffer, 1, i, c);
    }

    DMA_IO_SendBuffer(&DMA_IO_FrameBuffer, BUFFSIZE);

    d++;
    d %= 3000;
  }
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
