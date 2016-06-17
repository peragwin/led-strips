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
#include "usart.h"
#include "gpio.h"

#include "ControlRegisters.h"

/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "math.h"
#define PI 3.1415926
/* USER CODE END Includes */


/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define MAJOR_REV 0
#define MINOR_REV 1

// buffer size is num_leds * 24 bits/led
// 60 * 24 = 1440
#define BUFFSIZE 1440
#define NUM_CHANNELS 8
#define NUM_LEDS_PER_CHANNEL 60
#define MAX_NUM_CHANNELS 8

#define DEFAULT_BRIGHTNESS 50
#define DEFAULT_FRAMERATE  10


#include "FrameBuffer.h"

// put these in memory so they can be copied via DMA
uint32_t _rawBuffer[BUFFSIZE/4];  //make aligned
buf      DMA_IO_FrameBuffer = (buf) &_rawBuffer;
uint32_t DMA_IO_High = 0xFFFFFFFFU;
uint32_t DMA_IO_Low  = 0x00000000U;

volatile uint8_t WS2812_TransferComplete = 1;


extern TIM_HandleTypeDef htimx;
extern DMA_HandleTypeDef hdma_timx_gpio_low;     // DMA1_stream4 sets data gpios low
extern DMA_HandleTypeDef hdma_timx_gpio_data;    // DMA1_stream5 sets data gpios to buffer value
extern DMA_HandleTypeDef hdma_timx_gpio_high;    // DMA1_stream6 sets data gpios high
extern UART_HandleTypeDef huart2;





volatile int showFrameDebug = 1;
volatile int serialEchoEnabled = 1;
int globalBrightness = 50;
int globalFrameDelay = 100;





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void DebugGood_Handler(void);
void Debug_Setup(void);


void DMA_IO_SendBuffer(buf frameData, uint16_t buffsize);


void DisplayLedDemo(void);


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


GPIO_TypeDef *GpioC;

void Debug_Setup(void)
{
  GpioC = (GPIO_TypeDef*)GPIOC;

  // enable usasge error handler status reg
  *((uint32_t*)0xe000ed24) = 0x70000;
}

void DMA_IO_SendBuffer(buf frameBuffer, uint16_t buffsize)
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
                       (uint32_t)frameBuffer,
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


float triangleWave(int period, float amplitude, int phase, int d)
{
  int p = phase % 360;
  float t = (360.0/period * d) - p;

  if (t>=180) return amplitude * (-1 + (t-180)/90);
  else        return amplitude * (1 - t/90);
}

void RainbowShiftIterateFrame(buf fb, int d)
{
    int i;
    float w;
    Pixel c;
    static PixelBlock pixelBlockNext;

    c = ColorTable[(4*d)%COLOR_TABLE_LEN];
    for (i=0; i<NUM_CHANNELS; i++) {
     /* w = triangleWave(450, 4.0, 0, d+80*i);
      if (w > 1) w = 1.0/w;
        else if (w < -1) w = -w;
        else w = 1.0; */
      //w = (float)globalBrightness / 100.0;

      pixelBlockNext[i] = setPixelBrightness(c, globalBrightness);
    }

    for(i = 0; i < NUM_LEDS_PER_CHANNEL; i++)
      FB_ShiftSetPixel(fb, pixelBlockNext, i);

}

int frameCount = 0;

void DisplayLedDemo(void)
{
  // go for cycle period of 10 sec, or 300 frames
  //const float w = PI/300;

  buf fb = DMA_IO_FrameBuffer;

  while(frameCount++)
  {
    int d = frameCount;
    if (!(d%30) || d%30==6) HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    RainbowShiftIterateFrame(fb, d);

    DMA_IO_SendBuffer(fb, BUFFSIZE);
    HAL_Delay(globalFrameDelay);

  }
}

void ShowPrompt(void)
{
  char *prompt = "#\r\n%s> ";
  char *frame = "Frame %d ";
  if (showFrameDebug) {
    sprintf(frame, frame, frameCount);
    printf(prompt, frame);
  }
  else printf(prompt, "");
}


void ParseCommands(char *cmdStr)
{
  char cmd[16], value[8];
  char *csep, *cend;
  int cmdLen;

  csep = strchr(cmdStr, ' ');
  cend = strchr(cmdStr, '\r');
  if (!cend) cend = strchr(cmdStr, '\n');
  if (csep == NULL || cend == NULL) return;

  cmdLen = csep - cmdStr;
  strncpy(cmd, cmdStr, cmdLen);
  strncpy(value, csep+1, cend-csep-1);

  if (cmd[0] == 'b')
    globalBrightness = atoi(value);

  if (cmd[0] == 'r')
    globalFrameDelay = atoi(value);
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
  USART2_Init();
  HAL_UART_RxCpltCallback(&huart2); // begin rx for loopback

  Debug_Setup();

  /* USER CODE BEGIN 2 */

  for (uint16_t i = 0; i < NUM_LEDS_PER_CHANNEL; i++)
  {
    FB_SetPixel(DMA_IO_FrameBuffer, 0, i, CWhite);
  }

  uint16_t size = BUFFSIZE;
  while(0);
  DMA_IO_SendBuffer(DMA_IO_FrameBuffer, size);

  ShowPrompt();

  while (1)
  {

    while(!WS2812_TransferComplete)
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(500);
    }
    DebugGood_Handler();
    //DMA_IO_SendBuffer(DMA_IO_FrameBuffer, BUFFSIZE);

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

  DisplayLedDemo();

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
