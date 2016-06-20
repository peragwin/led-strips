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

#define ARM_MATH_CM4

#include "arm_math.h"
  #include "math.h"
//#define PI 3.1415926
/* USER CODE END Includes */


/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define MAJOR_REV 0
#define MINOR_REV 1

// buffer size is num_leds * 24 bits/led
// 60 * 24 = 1440
#define BUFFSIZE 2*1440
#define NUM_CHANNELS 8
#define NUM_LEDS_PER_CHANNEL 2*60
#define MAX_NUM_CHANNELS 8

#define DEFAULT_BRIGHTNESS 50
#define DEFAULT_FRAMERATE  10


#include "FrameBuffer.h"

// put these in memory so they can be copied via DMA
uint32_t _rawBuffer[2 * BUFFSIZE/4];  //make aligned
buf      DMA_IO_FrameBuffer = (buf) (&_rawBuffer);
buf      FrameBufferZero = (buf) (&_rawBuffer);
buf      FrameBufferOne = (buf)((uint8_t*)&_rawBuffer + BUFFSIZE);
uint32_t DMA_IO_High = 0xFFFFFFFFU;
uint32_t DMA_IO_Low  = 0x00000000U;

volatile uint8_t WS2812_TransferComplete = 1;


extern TIM_HandleTypeDef htimx;
extern TIM_HandleTypeDef htimFrame;
extern DMA_HandleTypeDef hdma_timx_gpio_low;     // DMA1_stream4 sets data gpios low
extern DMA_HandleTypeDef hdma_timx_gpio_data;    // DMA1_stream5 sets data gpios to buffer value
extern DMA_HandleTypeDef hdma_timx_gpio_high;    // DMA1_stream6 sets data gpios high
extern UART_HandleTypeDef huart2;





volatile int showFrameDebug = 1;
volatile int serialEchoEnabled = 1;

uint globalFrameDelay = 100;
uint globalBrightness = 100;
uint globalLedsPerChannel = NUM_LEDS_PER_CHANNEL;
int globalSubBrightness = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void DebugGood_Handler(void);
void Debug_Setup(void);


void DMA_IO_SendBuffer(buf frameData, uint16_t buffsize);


void DisplayLedDemo(void);

int serialDoRawRegisterRead(char* args);
int serialDoRawRegisterWrite(char* args);


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

  static int subBrightCn;
  subBrightCn++;
  uint32_t frameBufferOrZeros = (uint32_t) frameBuffer;
  int g = globalSubBrightness;
  if (g > 0 && g <= 8)
      if (!(subBrightCn % g))
          frameBufferOrZeros = &DMA_IO_Low;
  if (g < 0 && g >= -8)
      if (subBrightCn % g)
          frameBufferOrZeros = &DMA_IO_Low;


  // Enable DMA
  if (HAL_DMA_Start_IT(&hdma_timx_gpio_high,
                       (uint32_t)&DMA_IO_High,
                       (uint32_t)&(GPIOC->ODR),
                       buffsize)
      != HAL_OK) Error_Handler();

  if (HAL_DMA_Start_IT(&hdma_timx_gpio_data,
                       frameBufferOrZeros,
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
  __HAL_TIM_SET_COUNTER(&htimx, (uint16_t)(DATA_TIM_PULSE2+1));

  // Enable the timer to kick things off
  // __HAL_TIM_ENABLE_IT(&htimx, TIM_IT_UPDATE|TIM_IT_CC2); // ??
  __HAL_TIM_ENABLE(&htimx);

}


void (*frameUpdateFn)(void) = NULL;

void FrameUpdater()
{
  if (frameUpdateFn != NULL)
    frameUpdateFn();

  if(WS2812_TransferComplete)
    DMA_IO_SendBuffer(DMA_IO_FrameBuffer, BUFFSIZE);
}

void setFrameDelay(uint d)
{
  globalFrameDelay = d;
  __HAL_TIM_SET_AUTORELOAD(&htimFrame, d);
}

void frameDelay(void)
{
  //for(int i = 0; i < 40000*globalFrameDelay; i++);
  HAL_Delay(globalFrameDelay);
}

void enableFrameUpdates(void)
{
  __HAL_TIM_ENABLE_IT(&htimFrame, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE(&htimFrame);
}

void disableFrameUpdates(void)
{
  __HAL_TIM_DISABLE(&htimFrame);
  __HAL_TIM_DISABLE_IT(&htimFrame, TIM_IT_UPDATE);
}
















float triangleWave(int period, float amplitude, int phase, int d)
{
  int p = phase % 360;
  float t = (360.0/period * d) - p;

  if (t>=180) return amplitude * (-1 + (t-180)/90);
  else        return amplitude * (1 - t/90);
}










int frameCount = 0;

struct DemoOneConfig {
  uint16_t frameDelay;
  uint16_t period[MAX_NUM_CHANNELS];
  uint16_t phase[MAX_NUM_CHANNELS];
  uint16_t brightness[MAX_NUM_CHANNELS];
};

struct DemoOneConfig *demoOneConfig;



void rainbowShiftIterateFrame(buf fb, struct DemoOneConfig *config)
{
    int ch, period, phase;
    float brightness, w, red_, green_, blue_, s;
    uint8_t red, green, blue;
    Pixel c;
    static int d;
    static PixelBlock pixelBlockNext;

    //c = ColorTable[d%COLOR_TABLE_LEN];
    for (ch=0; ch < 3; ch++)
    {
      
      brightness = (float)config->brightness[ch];
      period =  config->period[ch];
      phase = config->phase[ch];
      w = PI / period;

      red_   = 128.0f + 120.0f*sinf((float32_t)w*(d + phase));
      green_ = 128.0f + 120.0f*sinf(w*(d + phase + 2*period/3));
      blue_  = 128.0f + 120.0f*sinf(w*(d + phase - 2*period/3));
      s = red_ + green_ + blue_;
      red = (uint8_t)round(red_ * brightness/s);
      green = (uint8_t)round(blue_ * brightness/s);
      blue = (uint8_t)round(green_ * brightness/s);

      c = (Pixel) { red, green, blue };
      //printf("%2x %2x %2x\r\n", c.red, c.blue, c.green); */
      pixelBlockNext[ch] = AdjustPixelBrightness(c);
    }

    FB_FastSetPixel(fb, pixelBlockNext, globalLedsPerChannel-1);
    FB_FastSetPixel(fb, pixelBlockNext, -1);
    //for(ibnt i = 0; i < globalLedsPerChannel; i++)
    //  FB_ShiftSetPixel(fb, pixelBlockNext, i);

    d++;
}

static volatile int frameReady = 1;

void doRainbowShiftIterateFrame(int fbOffset)
{

    rainbowShiftIterateFrame(FrameBufferZero+fbOffset, demoOneConfig);


}


void DisplayLedDemoOne(void)
{
  int fbOffset = 0;
  int buffsize;
  static struct DemoOneConfig config =  { 600, {100, 100, 100}, {0}, {80, 80, 80} };
  demoOneConfig = &config;
  setFrameDelay(config.frameDelay);
  //frameUpdateFn = doRainbowShiftIterateFrame;
  //enableFrameUpdates();
  while(1){
    buffsize = globalLedsPerChannel*24;
    fbOffset %= buffsize;
    fbOffset+=24;
    frameDelay();
    doRainbowShiftIterateFrame(fbOffset);
    DMA_IO_SendBuffer(FrameBufferZero+fbOffset, globalLedsPerChannel*24);
  }
}


int serialDoDemoOneSetBrightness(struct DemoOneConfig *config, char *args){
  char *bp;
  long bVal;
  long chVal;
  bVal = strtol(args, &bp, 10);
  if (*bp != 0) chVal = strtol(bp, &bp, 10);
  else chVal = -1;

  printf("DemoOne, set brightness[%d]: %d", chVal, bVal);

  if (chVal >= 0 && chVal < NUM_CHANNELS)
    config->brightness[chVal] = bVal;

  else
    for (int c = 0; c < NUM_CHANNELS; c++) config->brightness[c] = bVal;
  return 0;
}
int serialDoDemoOneSetPeriod(struct DemoOneConfig *config, char *args){
  char *bp;
  long bVal;
  long chVal;
  bVal = strtol(args, &bp, 10);
  if (*bp != 0) chVal = strtol(bp, &bp, 10);
  else chVal = -1;

  printf("DemoOne, set period[%d]: %d", chVal, bVal);

  if (chVal >= 0 && chVal < NUM_CHANNELS)
    config->period[chVal] = bVal;

  else
    for (int c = 0; c < NUM_CHANNELS; c++) config->period[c] = bVal;
  return 0;
}
int serialDoDemoOneSetPhase(struct DemoOneConfig *config, char *args){
  char *bp;
  long bVal;
  long chVal;
  bVal = strtol(args, &bp, 10);
  if (*bp != 0) chVal = strtol(bp, &bp, 10);
  else chVal = -1;

  printf("DemoOne, set phase[%d]: %d", chVal, bVal);

  if (chVal >= 0 && chVal < NUM_CHANNELS)
    config->phase[chVal] = bVal;

  else
    for (int c = 0; c < NUM_CHANNELS; c++) config->phase[c] = bVal;
  return 0;
}













uint verifyAddress(char *args, char **sp)
{
  long addrL = strtol(args, sp, 16);
  if (addrL < 0 || addrL > 0xffffu) {
    printf("Error: address out of range (%x)\n", (uint)addrL);
    return -1;
  }
  return (uint)addrL;
}

int serialDoRawRegisterRead(char* args)
{
  RawRegReadValue regRead;
  uint address;

  if ((address = verifyAddress(args, NULL)) == -1)
    return -1;

  RawRegisterRead(address, &regRead);
  if (regRead.flag == invalidAccess)
  {
    printf("Error: address %4x returned 'invalidAccess'\n", address);
    return 2;
  }

  printf("Read: %8x (%4x)\n", (uint)regRead.reg, address);
  return 0;
}

int serialDoRawRegisterWrite(char *args)
{
  char *sp;
  uint address;
  long wrVal;
  HandleRegAccessFlag f;

  if ((address = verifyAddress(args, &sp)) == -1)
    return -1;

  wrVal = strtol(sp, &sp, 16);

  f = RawRegisterWrite(address, (uint32_t)wrVal);
  switch (f)
  {
    case readOnlyAccess:
      printf("Error: address %4x returned 'readOnlyAccess'\n", address);
      return 4;
    case invalidAccess:
      printf("Error: address %4x returned 'invalidAccess'\n", address);
      return 2;
    default:
      printf("Wrote: %8x (%4x)\n", (uint)wrVal, address);
      return 0;
  }
}












void ShowPrompt(void)
{
  char prompt[32] = { 0 };
  char frame[16] = { 0 };

  strncpy(prompt, "\r\n%s> ", sizeof(prompt));
  strncpy(frame, "Frame %d ", sizeof(frame));

  if (showFrameDebug) {
    sprintf(frame, frame, frameCount);
    printf(prompt, frame);
  }
  else printf(prompt, "");
}


void ParseCommands(char *cmdStr)
{
  char cmd[16], args[32];
  char *csep, *cend;
  int cmdLen, fd;
  int rv = 0;

  csep = strchr(cmdStr, ' ');
  cend = strchr(cmdStr, '\r');
  if (!cend) cend = strchr(cmdStr, '\n');
  if (csep == NULL || cend == NULL) return;

  cmdLen = csep - cmdStr;
  strncpy(cmd, cmdStr, cmdLen);
  strncpy(args, csep+1, cend-csep-1);

  switch (cmd[0])
  {

    case 's':
      if (cmd[1] == 'u' && cmd[2] == 'b')
      { // sub brightness
        int g = atoi(args);
        if (!(g < -8 || g > 8))
          globalSubBrightness = g;
      }
      else
        // here 'args' should be a valid "addr value"
        rv = serialDoRawRegisterWrite(args);
      break;

    case 'g':
      rv = serialDoRawRegisterRead(args); break;

    // these cases to be customized by __current_demo__

    case 'b':
      rv = serialDoDemoOneSetBrightness(demoOneConfig, args); break;

    case 'c':
      if (cmd[1] == 'h' && cmd[2] == 'a')
      {
        int g = atoi(args);
        if (g >= 0 && g <= 240)
          globalLedsPerChannel = g;
      }
      break;

    case 'f':
      fd = atoi(args);
      printf("Set frameDelay: %d", fd);
      setFrameDelay((uint)fd);
      break;

    case 'p':
      if (cmd[1] == 'h') serialDoDemoOneSetPhase(demoOneConfig, args);
      else serialDoDemoOneSetPeriod(demoOneConfig, args);
      break;

    default:
      rv = -1;
  }

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
  TIM2_Init();
  //MX_TIM_FRAME_Init();
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

  InitControlRegisters();
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

  DisplayLedDemoOne();
  for(;;);

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
