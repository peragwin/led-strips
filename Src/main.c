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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"
#include "adc.h"

#include "ControlRegisters.h"
#include "FrameBuffer.h"

#include "stm32f4xx_it.h"




// put these in memory so they can be copied via DMA
uint32_t _rawBuffer[2 * BUFFSIZE/4];  //make aligned
buf      DMA_IO_FrameBuffer = (buf) (&_rawBuffer);
buf      FrameBufferZero = (buf) (&_rawBuffer);
buf      FrameBufferOne = (buf)((uint8_t*)&_rawBuffer + BUFFSIZE);
uint32_t DMA_IO_High = 0xFFFFFFFFU;
uint32_t DMA_IO_Low  = 0x00000000U;

volatile uint8_t WS2812_TransferComplete = 1;

extern uint8_t spiTxBuffer[SPI_BUFFSIZE];
extern uint8_t spiRxBuffer[SPI_BUFFSIZE];


extern TIM_HandleTypeDef htimx;
extern TIM_HandleTypeDef htim_dead;
extern DMA_HandleTypeDef hdma_timx_gpio_low;     // DMA1_stream4 sets data gpios low
extern DMA_HandleTypeDef hdma_timx_gpio_data;    // DMA1_stream5 sets data gpios to buffer value
extern DMA_HandleTypeDef hdma_timx_gpio_high;    // DMA1_stream6 sets data gpios high
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;


uint16_t _raw_ADC_Buffer[ADC_BUFFER_LENGTH+2];
uint16_t *ADC_Buffer = _raw_ADC_Buffer+1;

volatile int showFrameDebug = 1;
volatile int serialEchoEnabled = 1;


extern struct ControlRegisters controlRegisters;

volatile u8 *gPause = &controlRegisters.displayStatus.pause;
volatile u16 *gFrameDelay = &controlRegisters.displayStatus.frameDelay;
volatile u16 *gBrightness = &controlRegisters.displayStatus.brightness;
volatile u8 *gNumLedsPerChannel = &controlRegisters.displayControl.numberLedsPerChannel;
volatile u8 *gNumberOfChannels = &controlRegisters.displayControl.numberOfChannels;
volatile u8 *gDisplayMode = &controlRegisters.displayMode;
int gSubBrightness = 0;

int (*currentModeSerialCommandPlugin)(char*, char*);

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













/*
GPIO_TypeDef *GpioC;

void Debug_Setup(void)
{
  GpioC = (GPIO_TypeDef*)GPIOC;

  // enable usasge error handler status reg
  *((uint32_t*)0xe000ed24) = 0x70000;
} */

void DMA_IO_SendBuffer(buf frameBuffer, uint16_t buffsize)
{
  while(!WS2812_TransferComplete) HAL_Delay(1);
  WS2812_TransferComplete = 0;

  // Stop ADC conversions to avoid cross noise and request conflicts.
  HAL_ADC_Stop_DMA(&hadc1);

  uint32_t frameBufferOrZeros = (uint32_t) frameBuffer;

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
void FrameXferCompleteCallback(DMA_HandleTypeDef *hdma){
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buffer, ADC_BUFFER_LENGTH) != HAL_OK)
      Error_Handler();

  int once = 0;
  while((htimx.Instance->CNT < DATA_TIM_PULSE2) || once<20) once++; //wait for clock to trigger gpios LOW

  // a full __HAL_TIM_DISABLE takes too long!
  (htimx.Instance)->CR1 &= ~(TIM_CR1_CEN);
  __HAL_TIM_DISABLE(&htimx);
  GPIOC->ODR = 0x0000;


  // Disable DMA
  __HAL_DMA_DISABLE(&hdma_timx_gpio_low);
  __HAL_DMA_DISABLE(&hdma_timx_gpio_high);
  __HAL_DMA_DISABLE(&hdma_timx_gpio_data);
  __HAL_TIM_DISABLE_DMA(&htimx, TIM_DMA_UPDATE);
  __HAL_TIM_DISABLE_DMA(&htimx, TIM_DMA_CC1);
  __HAL_TIM_DISABLE_DMA(&htimx, TIM_DMA_CC2);

  // enable TIM2 UPDATE interrupt to ensure wait 50us
  __HAL_TIM_ENABLE_IT(&htim_dead, TIM_IT_UPDATE); 
  __HAL_TIM_ENABLE(&htim_dead);

}



void setFrameDelay(uint d)
{
  *gFrameDelay = d;
}

void frameDelay(void)
{
  //for(int i = 0; i < 40000*globalFrameDelay; i++);
  HAL_Delay(*gFrameDelay);
}




























































#define FFT_RATIO 1
#define FFT_LENGTH ADC_BUFFER_LENGTH*FFT_RATIO
#define NUM_FFT_FILTERS 4 // only 4 channels connected right now

float fftIn[FFT_LENGTH];
float fftOut[FFT_LENGTH];
float fftFilter[FFT_LENGTH] = {0};

int fftFilterLength[NUM_FFT_FILTERS] = { 4*FFT_RATIO, 6*FFT_RATIO, 8*FFT_RATIO,
    FFT_LENGTH - 1 - 4*FFT_RATIO - 6*FFT_RATIO - 8*FFT_RATIO};
float fftEqValue[NUM_FFT_FILTERS] = { 0.5, 0.8, 2.0, 4.5 };


float audioEffectScale = 0.5;
float audioDiffEffectScale = 0.5;
float longAudioEffectScale = 0.85;

float longAudioEffect[NUM_FFT_FILTERS];

float fftIntensity[NUM_FFT_FILTERS];

arm_rfft_fast_instance_f32 fftStruct;


float fftWindow[FFT_LENGTH];
float confinedGaussian(float n) {
  #define G_SIGMA 0.1
  float wGaussian(float a){
    float aa = ( a - ((FFT_LENGTH-1)/2.0) )/( 2.0*G_SIGMA*FFT_LENGTH );
    return exp( -(aa*aa) );
  }
  float g1 = wGaussian(-0.5);
  float g2 = wGaussian(n + FFT_LENGTH) + wGaussian(n - FFT_LENGTH);
  float g4 = wGaussian(-0.5 + FFT_LENGTH) + wGaussian(-0.5 - FFT_LENGTH);
  return wGaussian(n) - g1 * g2 / g4;
}


#define IIR_FILTER_ORDER 4

float audioEffect[NUM_FFT_FILTERS][IIR_FILTER_ORDER];
float audioDiffEffect[NUM_FFT_FILTERS][1];

float filterParams[NUM_FFT_FILTERS][IIR_FILTER_ORDER][2] = {
  { 
    { 0.5, 0.5 }, // first order
    { -0.005, 0.995 }, // second order
    { 0.0, 0.0 }, // third order
    { 0.0, 0.0 } // fourth order
  },
  { 
    { 0.8, 0.5 }, // first order
    { -0.005, 0.995 }, // second order
    { 0.0, 0.0 }, // third order
    { 0.0, 0.0 } // fourth order
  },
  { 
    { 1.2, 0.5 }, // first order
    { -0.005, 0.995 }, // second order
    { 0.0, 0.0 }, // third order
    { 0.0, 0.0 } // fourth order
  },
  { 
    { 10.0, 0.5 }, // first order
    { -0.005, 0.995 }, // second order
    { 0.0, 0.0 }, // third order
    { 0.0, 0.0 } // fourth order
  }
};
float diffFilterParams[NUM_FFT_FILTERS][1][2] = {
  {
    { 1.0, 0.5 }
  },
  {
    { 1.0, 0.5 }
  },
  {
    { 1.0, 0.5 }
  },
  {
    { 1.0, 0.5 }
  }
};

void updateAudioFilters(void) {
  int filterOffset = 1; // ignore dc component
  for (int i = 0; i < NUM_FFT_FILTERS; i++){
    arm_fill_f32(1.0/fftFilterLength[i], fftFilter+filterOffset, fftFilterLength[i]);
    filterOffset += fftFilterLength[i];
  }
}

void initAudioProcessing(void) {
  for (int i = 0; i < FFT_LENGTH; i++) {
    fftIn[i] = 0;
    fftOut[i] = 0;
    fftWindow[i] = confinedGaussian((float)i);
  }
  arm_rfft_fast_init_f32(&fftStruct, FFT_LENGTH);
  updateAudioFilters();
}


void applyIIRFilter(
  float (*filterEq)[2], // [filterOrder] x [2]
  float input,
  float *output, // [filterOrder]
  int filterOrder,
  float *dInput
) {
  int order;
  float ae;
  for (order = 0; order < filterOrder; order++) {
    ae = filterEq[order][0] * input + filterEq[order][1] * output[order];
    if (dInput != NULL)
      dInput[order] = ae - output[order];

    input = ae;
    output[order] = ae;
  }
  for (order = filterOrder - 1; order > 0; order--) {
    output[order - 1] += output[order] - 0.0001; // correct for fp precision in negative feedback
  }
}


static int callbacksPerFrame = 0;
static int ncallback = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  int i, filterOffset, fftOffset;

  fftOffset = ADC_BUFFER_LENGTH * (ncallback % FFT_RATIO);
  for (i = 0; i < ADC_BUFFER_LENGTH; i++) {
    fftIn[i + fftOffset] = (float) ADC_Buffer[i];
  }
  arm_mult_f32(fftIn+fftOffset, fftWindow+fftOffset, fftIn+fftOffset, ADC_BUFFER_LENGTH);

  if (ncallback % FFT_RATIO == 0) {
    int fnum;
    float dInput[IIR_FILTER_ORDER];
    
    arm_rfft_fast_f32(&fftStruct, fftIn, fftOut, 0);
    arm_abs_f32(fftOut, fftOut, FFT_LENGTH);

    filterOffset = 1; // ignore dc component
    for (i = 0; i < NUM_FFT_FILTERS; i++){
      arm_dot_prod_f32(fftOut+filterOffset, fftFilter+filterOffset, fftFilterLength[i], &fftIntensity[i]);
      filterOffset += fftFilterLength[i];
    }

    for(fnum = 0; fnum < NUM_FFT_FILTERS; fnum++) {
      // filterEq = filterParams[fnum];
      // input = fftIntensity[fnum];
      // output = audioEffect[fnum];
      applyIIRFilter(
        filterParams[fnum], fftIntensity[fnum], audioEffect[fnum], IIR_FILTER_ORDER, dInput);

      applyIIRFilter(
        diffFilterParams[fnum], dInput[0], audioDiffEffect[fnum], 1, NULL);
    }
  }

  callbacksPerFrame++;
  ncallback++;
}

// expects "eq %d %d %d %f"
int setEqValue(char *args){
  float eqVal;
  int filterNum, filterOrder, eqIdx, i;
  char *p = args;

  filterNum = (int) strtol(p, &p, 10);
  filterOrder = (int) strtol(p, &p, 10);
  eqIdx = (int) strtol(p, &p, 10);
  eqVal = strtof(p, NULL);

  if (filterNum < -1 || filterNum >= NUM_FFT_FILTERS) return -2;
  if (filterOrder < 0 || filterOrder >= IIR_FILTER_ORDER) return -3;
  if (eqIdx < 0 || eqIdx > 1) return -4;

  if (filterNum == -1)
    for (i = 0; i < NUM_FFT_FILTERS; i++)
      filterParams[i][filterOrder][eqIdx] = eqVal;

  else
    filterParams[filterNum][filterOrder][eqIdx] = eqVal;

  printf("Set eqValue for filter[%d], order[%d], idx[%d] = %.4f\r\n",
      filterNum, filterOrder, eqIdx, eqVal);

  return 0;
}

int setDiffEqValue(char *args){
  float eqVal;
  int filterNum, eqIdx, i;
  char *p = args;

  filterNum = (int) strtol(p, &p, 10);
  eqIdx = (int) strtol(p, &p, 10);
  eqVal = strtof(p, NULL);

  if (filterNum < -1 || filterNum >= NUM_FFT_FILTERS) return -2;
  if (eqIdx < 0 || eqIdx > 1) return -3;

  if (filterNum == -1)
    for (i = 0; i < NUM_FFT_FILTERS; i++)
      diffFilterParams[i][0][eqIdx] = eqVal;

  else
    diffFilterParams[filterNum][0][eqIdx] = eqVal;

  printf("Set diff eqValue for filter[%d], idx[%d] = %.4f\r\n",
      filterNum, eqIdx, eqVal);

  return 0;
}

int setFilterSize(char *args){
  int fSize, fIdx, oldSize, newHighSize;

  fIdx = (int) strtol(args, &args, 10);
  fSize = (int) strtol(args, NULL, 10);

  if (fIdx < 0 || fIdx >= NUM_FFT_FILTERS) return -2;

  oldSize = fftFilterLength[fIdx];
  fftFilterLength[fIdx] = fSize;
  newHighSize = FFT_LENGTH - fftFilterLength[0] - fftFilterLength[1] - fftFilterLength[2];
  
  if (newHighSize < 0) {
    fftFilterLength[fIdx] = oldSize;
    return -3;
  }

  printf("Set filterSize[%d] = %d\r\n", fIdx, fSize);
  updateAudioFilters(); // probably not needed

  return 0;
}

void printFilterValues(void) {
  int i, j;
  //HAL_Delay(1); // dont overrun print buffer
  for (i = 0; i < NUM_FFT_FILTERS; i++) {
    printf("\r\nFilter %d:\r\n", i);
    printf("- size: %d\r\n", fftFilterLength[i]);
    printf("- eqValues:\r\n");
    for (j = 0; j < IIR_FILTER_ORDER; j++) {
      printf("\t- order %d: %.4f, %.4f\r\n", j, filterParams[i][j][0], filterParams[i][j][1]);
    }
    printf("- diffEqValues: %.4f, %.4f\r\n", diffFilterParams[i][0][0], diffFilterParams[i][0][1]);
  }
}

int audioSerialCommandPlugin(char *cmd, char *args) {
  int rv = -1;
  switch (cmd[0]) {
    case 'f':
      if (cmd[1] == 's')
        rv = setFilterSize(args);
      break;

    case 'e':
      if (cmd[1] == 'q')
        rv = setEqValue(args);
      break;

    case 'd':
      if (cmd[1] == 'e' && cmd[2] == 'q')
        rv = setDiffEqValue(args);
      break;

    default:
      rv = -1;
  }
  return rv;
}












/*
float triangleWave(int period, float amplitude, int phase, int d)
{
  int p = phase % 360;
  float t = (360.0/period * d) - p;

  if (t>=180) return amplitude * (-1 + (t-180)/90);
  else        return amplitude * (1 - t/90);
}
*/











































/*****          ********          ********       *********       ******/
/* BEGIN DEMO ONE STUFF                                               */
/************               *****************                  ********/

struct DemoOneConfig
{
  uint16_t timeScale[MAX_NUM_CHANNELS];
  uint16_t timeIncrement[MAX_NUM_CHANNELS];
  //uint8_t modUpdate[MAX_NUM_CHANNELS];
  uint16_t amp[MAX_NUM_CHANNELS];
  uint16_t ampOffset[MAX_NUM_CHANNELS];
  uint16_t brightness[MAX_NUM_CHANNELS];
  uint16_t timePeriod[MAX_NUM_CHANNELS];
  uint16_t spacePeriod[MAX_NUM_CHANNELS];
  //uint16_t phase[MAX_NUM_CHANNELS];
  float (*iterFunc)(float);
};
typedef struct DemoOneConfig conf1;

int demoOneSerialCommandPlugin(char *cmd, char *args);

void DisplayLedDemoOne(void)
{
  conf1 *newDemoOneConfig()
  {
    static conf1 conf;
    for (int i = 0; i < MAX_NUM_CHANNELS; i++)
    {
      conf.timeScale[i] = 6000;
      conf.timeIncrement[i] = 1;
     // conf.modUpdate[i] = 1;
      conf.amp[i] = 1;
      conf.ampOffset[i] = 800;
      conf.brightness[i] = 800;
      conf.timePeriod[i] = 300;
      conf.spacePeriod[i] = 100;//MAX_NUM_LEDS_PER_CHANNEL;
     // conf.phase[i] = 0;
    }
    conf.iterFunc = arm_sin_f32;
  return &conf;
  }

  conf1 *config = newDemoOneConfig();
  controlRegisters.demoConfig = config;
  currentModeSerialCommandPlugin = demoOneSerialCommandPlugin;

  static float d[MAX_NUM_CHANNELS] = {0};
  static int ucount = 0;
  uint8_t nLedsPerCh = *gNumLedsPerChannel;
  void rainbowShiftIterateFrame(buf fb, conf1 *config, float (*iterFunc)(float)) {
    int ch, px;
    float timeperiod, spaceperiod, br, wt, ws, red_, green_, blue_, s;
    float amp[MAX_NUM_CHANNELS];
    uint8_t red, green, blue;
    Pixel c;
    PixelBlock pixelBlock;
    uint8_t numChan = *gNumberOfChannels;
    nLedsPerCh = *gNumLedsPerChannel;
    uint16_t gBr = *gBrightness;
    bool updateAny = true;

    ch = 0;
    //amp = (float)config->amp[ch];
    br = (float)config->brightness[ch];
    timeperiod = (float)config->timePeriod[ch];
    spaceperiod = (float)config->spacePeriod[ch];
    wt = 2.0f * PI / timeperiod;
    ws = 2.0f * PI / spaceperiod;
    
    for (ch = 3; ch < 7; ch++) {
    //  if (ucount % config->modUpdate[ch] == 0) {
        d[ch] -= audioDiffEffect[ch-3][0] * (float)config->timeIncrement[ch] / (float)config->timeScale[ch];
    //    updateAny = true;
    }
    ucount++;

    for (ch = 3; ch < 7; ch++)
      amp[ch] = (float)config->ampOffset[ch] + ((float)config->amp[ch] * audioEffect[ch-3][0]);

    if (updateAny)
      for (px = 0; px < nLedsPerCh; px++) {
        for (ch = 3; ch < 7; ch++) {
        //  if (ucount % config->modUpdate[ch] == 0) {

        red_   = br + amp[ch] * iterFunc(  (wt * d[ch]) + (ws * px)  );
        green_ = br + amp[ch] * iterFunc(  (wt * d[ch]) + (ws * px) + 2.0*PI/3  );
        blue_  = br + amp[ch] * iterFunc(  (wt * d[ch]) + (ws * px) - 2.0*PI/3  );
        
        s = (float) gBr / (red_ + green_ + blue_);
        red_ *= s;
        green_ *= s;
        blue_ *= s;

        red = (uint8_t) round(  red_ > 255 ? 255 : ( red_ < 0 ? 0 : red_ )  );
        blue = (uint8_t) round(  blue_ > 255 ? 255 : ( blue_ < 0 ? 0 : blue_ )  );
        green = (uint8_t) round(  green_ > 255 ? 255 : ( green_ < 0 ? 0 : green_ )  );

        c = (Pixel) { red, green, blue };
        // if (px == 10) {
        //   printf("pixel %d: %2x %2x %2x, d = %.2f\r\n", px, red, green, blue, d[ch]);
        // }

       //for (ch = 0; ch < numChan; ch++)
          pixelBlock[ch] = c;
        }


        FB_FastSetPixel(fb, pixelBlock, px);
      }
  }





// demo main

  int buffsize;
  while (true) {
    if (*gPause) { HAL_Delay(200); continue; }
    //printf("cbfp: %d\r\n", callbacksPerFrame);
    callbacksPerFrame = 0;
    frameDelay();
    rainbowShiftIterateFrame(FrameBufferZero, config, config->iterFunc);
    buffsize = min(MAX_NUM_LEDS_PER_CHANNEL, nLedsPerCh)*24;
    DMA_IO_SendBuffer(FrameBufferZero, buffsize);
  }



}

int doDemoOneSetConfig(uint16_t *configVal, char *args, char *name){
  char *bp;
  long bVal;
  long chVal;
  bVal = strtol(args, &bp, 10);
  if (*bp != 0) chVal = strtol(bp, &bp, 10);
  else chVal = -1;

  printf("DemoOne, set %s[%d]: %d", name, (int)chVal, (int)bVal);

  if (chVal >= 0 && chVal < MAX_NUM_CHANNELS)
    configVal[chVal] = bVal;
  else
    for (int c = 0; c < MAX_NUM_CHANNELS; c++) configVal[c] = bVal;

  return 0;
}

int doDemoOneSetConfig8(uint8_t *configVal, char *args, char *name){
  char *bp;
  long bVal;
  long chVal;
  bVal = strtol(args, &bp, 10);
  if (*bp != 0) chVal = strtol(bp, &bp, 10);
  else chVal = -1;

  printf("DemoOne, set %s[%d]: %d", name, (int)chVal, (int)bVal);

  if (chVal >= 0 && chVal < MAX_NUM_CHANNELS)
    configVal[chVal] = bVal;
  else
    for (int c = 0; c < MAX_NUM_CHANNELS; c++) configVal[c] = bVal;

  return 0;
}

int demoOneSerialCommandPlugin(char *cmd, char *args) {
  int rv = -1;
  conf1 *demoConfig = (conf1*) controlRegisters.demoConfig;
  switch(cmd[0]) {
    case 'b':
      rv = doDemoOneSetConfig(
        demoConfig->brightness, args, "brightness");
      break;

    case 'a':
      if (cmd[1] == 'o')
        rv = doDemoOneSetConfig(
          demoConfig->ampOffset, args, "ampOffset");
      else
        rv = doDemoOneSetConfig(
          demoConfig->amp, args, "amplitude");
      break;

    case 'p':
      if (cmd[1] == 's') rv = doDemoOneSetConfig(
        demoConfig->spacePeriod, args, "spacePeriod");
      else rv = doDemoOneSetConfig(
        demoConfig->timePeriod, args, "timePeriod");
      break;

    case 't':
      if (cmd[1] == 'i') rv = doDemoOneSetConfig(
        demoConfig->timeIncrement, args, "timeIncrement");
      else if (cmd[1] == 's') rv = doDemoOneSetConfig(
        demoConfig->timeScale, args, "timeScale");
      break;

    case 's':
      printf("brightness: %d\r\n", demoConfig->brightness[0]);
      printf("amplitude: %d\r\n", demoConfig->amp[0]);
      printf("spacePeriod: %d\r\n", demoConfig->spacePeriod[0]);
      printf("timePeriod: %d\r\n", demoConfig->timePeriod[0]);
      printf("timeScale: %d\r\n", demoConfig->timeScale[0]);
      printFilterValues();
      rv = 0;
      break;

    default:
      rv = -1;
  }
//  if (rv == -1)
//    rv = audioSerialCommandPlugin(cmd, args);

  return rv;
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










const char modeStrings[] = "demoOne\0led\0";

void ShowPrompt(void)
{
  char mode[16]; 
  if (*gDisplayMode == 1)
    strncpy(mode, modeStrings, 8);
  else
    strncpy(mode, modeStrings+8, 4);
  printf("\r\n %s > ", mode);
  fflush(stdout);
}


int ParseCommands(char *cmdStr)
{
  char cmd[16] = {0};
  char args[32] = {0};
  char *csep, *cend;
  int cmdLen, fd;
  int rv = 0;

  csep = strchr(cmdStr, ' ');
  cend = strchr(cmdStr, '\r');
  if (!cend) cend = strchr(cmdStr, '\n');
  if (!cend) cend = strchr(cmdStr, 0);
  if (csep == NULL || cend == NULL) return -1;
  //if (cend == NULL) return -1;
  //if (csep == NULL)
  //  csep = cend;

  cmdLen = csep - cmdStr;
  strncpy(cmd, cmdStr, cmdLen);
  strncpy(args, csep+1, cend-csep-1);

  switch (cmd[0])
  {

    case 'w':
      // here 'args' should be a valid "addr value"
      if (cmd[1] == 'r')
        rv = serialDoRawRegisterWrite(args);
      else rv = -1;
      break;

    case 'r':
      if (cmd[1] == 'd')
        rv = serialDoRawRegisterRead(args);
      else rv = -1;
      break;

    case 'f':
      if (cmd[1] == 0) {
        fd = atoi(args);
        printf("Set frameDelay: %d", fd);
        setFrameDelay((uint)fd);
      } else rv = -1;
      break;

    case 'g':
      fd = atoi(args);
      printf("Set gBrightness: %d", fd);
      *gBrightness = fd;
      break;

    case 'c':
      if (cmd[1] == 'h' && cmd[2] == 'a')
      {
        int g = atoi(args);
        if (g >= 0 && g <= 240)
          *gNumLedsPerChannel = g;
      }
      break;

    // these cases to be customized by __current_demo__

    default:
      rv = -1;

  }

  if (rv == -1)
    rv = currentModeSerialCommandPlugin(cmd, args);
  if (rv == -1)
    rv = audioSerialCommandPlugin(cmd, args);

  return rv;
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

  NVIC_Init();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  TIM_Leds_Init();
  TIM2_Init();

  USART2_Init();
  USART6_Init();
  HAL_UART_RxCpltCallback(&huart2); // begin rx for loopback
  HAL_UART_RxCpltCallback(&huart6);

  MX_SPI2_Init();
  if(HAL_SPI_TransmitReceive_DMA(&hspi2,
    (uint8_t*)spiTxBuffer,
    (uint8_t*)spiRxBuffer,
    SPI_BUFFSIZE) != HAL_OK) Error_Handler();

  MX_ADC1_Init();
  initAudioProcessing();
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buffer, ADC_BUFFER_LENGTH) != HAL_OK)
    Error_Handler();

  /* USER CODE BEGIN 2 */

  for (uint16_t i = 0; i < NUM_LEDS_PER_CHANNEL; i++)
  { 
    for (uint ch=0; ch<NUM_CHANNELS; ch++)
      FB_SetPixel(DMA_IO_FrameBuffer, ch, i, CWhite);
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
