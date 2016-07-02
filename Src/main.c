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

float audioEffect[4];
float audioDiffEffect[4];

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








#define FFT_LENGTH ADC_BUFFER_LENGTH/2
#define NUM_FFT_FILTERS 4 // only 4 channels connected right now

float fftIn[FFT_LENGTH];
float fftOut[FFT_LENGTH];
float fftFilter[FFT_LENGTH];

int fftFilterLength[NUM_FFT_FILTERS] = { 4, 8, 12, FFT_LENGTH - 1 - 4 - 4 - 12};
float fftEqValue[NUM_FFT_FILTERS] = { 0.5, 0.8, 2.0, 4.5 };

float audioEffectScale = 0.5;
float audioDiffEffectScale = 0.5;

float fftIntensity[4];


void updateAudioFilters(void) {
  int filterOffset = 1; // ignore dc component
  for (int i = 0; i < NUM_FFT_FILTERS; i++){
    arm_fill_f32(fftEqValue[i] / fftFilterLength[i], fftFilter+filterOffset, fftFilterLength[i]);
    filterOffset += fftFilterLength[i];
  }
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  //if (WS2812_TransferComplete == 0) return;
  int i, sum, filterOffset;
  float dIntensity;
  arm_rfft_fast_instance_f32 S;
  arm_rfft_fast_init_f32(&S, FFT_LENGTH);

  /* kill dc component
  sum = 0;
  for (i = 0; i < ADC_BUFFER_LENGTH; i++) {
    sum += ADC_Buffer[i];
  }
  mean = (float) sum / ADC_BUFFER_LENGTH;
  */

  // and average / decimate by 4
  for (i = 0; i < FFT_LENGTH; i++) {
    fftIn[i] =
      (float) (ADC_Buffer[2*i-1] + 2*ADC_Buffer[2*i] + ADC_Buffer[2*i+1]);
      //  + ADC_Buffer[i+1] + ADC_Buffer[i+2] + ADC_Buffer[i+3])
      //- 4.0 * mean;
  }

  arm_rfft_fast_f32(&S, fftIn, fftOut, 0);

  arm_abs_f32(fftOut, fftOut, FFT_LENGTH);

  filterOffset = 1; // ignore dc component
  for (i = 0; i < NUM_FFT_FILTERS; i++){
    arm_dot_prod_f32(fftOut+filterOffset, fftFilter+filterOffset, fftFilterLength[i], &fftIntensity[i]);
    filterOffset += fftFilterLength[i];
  }

  for(i = 0; i < 4; i++) {
    dIntensity = fftIntensity[i] - audioEffect[i];
    audioEffect[i] += audioEffectScale * dIntensity;
    audioDiffEffect[i] = (dIntensity - audioDiffEffect[i]) * audioDiffEffectScale;
  }
}

// expects "eq \d %f"
int setEqValue(char *args){
  float eqVal;
  int eqIdx;
  char* eqKey = args;
  bool shortKey = args[1] == ' ';
  char *p = args + (shortKey ? 2 : 3);

  eqVal = strtof(p, NULL);

  if (*eqKey == 'l')
    if (shortKey) eqIdx = 0;
    else if (eqKey[1] == 'm') eqIdx = 1;
    else return -1;

  else if (*eqKey == 'h')
    if (shortKey) eqIdx = 3;
    else if (eqKey[1] == 'm') eqIdx = 2;
    else return -1;

  else return -1;

  fftEqValue[eqIdx] = eqVal;
  printf("set eq value [%d]: %f", eqIdx, eqVal);
  updateAudioFilters();

  return 0;
}

int setFilterSize(char *args){
  int fSize, fIdx, oldSize, newHighSize;

  if (args[0] == 'l') fIdx = 0;
  else if (args[0] == 'm') fIdx = 1;
  else if (args[0] == 'h') fIdx = 2;
  else return -1;

  fSize = (int) strtol(args+2, NULL, 10);

  oldSize = fftFilterLength[fIdx];
  fftFilterLength[fIdx] = fSize;
  newHighSize = FFT_LENGTH - fftFilterLength[0] - fftFilterLength[1] - fftFilterLength[2];
  
  if (newHighSize < 0) {
    fftFilterLength[fIdx] = oldSize;
    return -1;
  }

  printf("set filter size [%d]: %d\r\n", fIdx, fSize);
  updateAudioFilters();

  return 0;
}

int setAudioEffectValue(char *args){
  float aeVal = strtof(args, NULL);
  audioEffectScale = aeVal;
  return 0;
}

int setAudioDiffEffectValue(char *args){
  float adeVal = strtof(args, NULL);
  audioDiffEffectScale = adeVal;
  return 0;
}














float triangleWave(int period, float amplitude, int phase, int d)
{
  int p = phase % 360;
  float t = (360.0/period * d) - p;

  if (t>=180) return amplitude * (-1 + (t-180)/90);
  else        return amplitude * (1 - t/90);
}








/*****          ********          ********       *********       ******/
/* BEGIN DEMO ONE STUFF                                               */
/************               *****************                  ********/

struct DemoOneConfig
{
  uint16_t timeScale[MAX_NUM_CHANNELS];
  uint16_t timeIncrement[MAX_NUM_CHANNELS];
  //uint8_t modUpdate[MAX_NUM_CHANNELS];
  uint16_t amp[MAX_NUM_CHANNELS];
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
      conf.timeScale[i] = 120;
      conf.timeIncrement[i] = 1;
     // conf.modUpdate[i] = 1;
      conf.amp[i] = 1;
      conf.brightness[i] = 8000;
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
        d[ch] -= audioDiffEffect[ch-3] * (float)config->timeIncrement[ch] / (float)config->timeScale[ch];
    //    updateAny = true;
    }
    ucount++;

    for (ch = 3; ch < 7; ch++)
      amp[ch] = (float)config->amp[ch] * audioEffect[ch-3];

    if (updateAny)
      for (px = 0; px < nLedsPerCh; px++) {
        for (ch = 3; ch < 7; ch++) {
        //  if (ucount % config->modUpdate[ch] == 0) {

        red_   = br + amp[ch] * iterFunc((wt * d[ch]) + (ws * px));
        green_ = br + amp[ch] * iterFunc((wt * d[ch]) + (ws * px) + 2.0*PI/3);
        blue_  = br + amp[ch] * iterFunc((wt * d[ch]) + (ws * px) - 2.0*PI/3);
        s = (float) gBr / (red_ + green_ + blue_);
        red = (uint8_t) round(red_ * s);
        green = (uint8_t) round(blue_ * s);
        blue = (uint8_t) round(green_ * s);

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

  int buffsize;
  while (true) {
    if (*gPause) { HAL_Delay(200); continue; }
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
  switch(cmd[0]) {
    case 'b':
      rv = doDemoOneSetConfig(
        ((conf1*) controlRegisters.demoConfig)->brightness, args, "brightness");
      break;

    case 'a':
      if (cmd[1] == 'e')
        rv = setAudioEffectValue(args);
      else if (cmd[1] == 'd' && cmd[2] == 'e')
        rv = setAudioDiffEffectValue(args);
      else
        rv = doDemoOneSetConfig(
          ((conf1*) controlRegisters.demoConfig)->amp, args, "amplitude");
      break;

    case 'p':
      if (cmd[1] == 's') rv = doDemoOneSetConfig(
        ((conf1*) controlRegisters.demoConfig)->spacePeriod, args, "spacePeriod");
      else rv = doDemoOneSetConfig(
        ((conf1*) controlRegisters.demoConfig)->timePeriod, args, "timePeriod");
      break;

    case 't':
      if (cmd[1] == 'i') rv = doDemoOneSetConfig8(
        ((conf1*) controlRegisters.demoConfig)->timeIncrement, args, "timeIncrement");
      else if (cmd[1] == 's') rv = doDemoOneSetConfig8(
        ((conf1*) controlRegisters.demoConfig)->timeScale, args, "timeScale");
      break;

    case 'e':
      if (cmd[1] == 'q') rv = setEqValue(args);
      break;

    case 'f':
      if (cmd[1] == 's') rv = setFilterSize(args);
      break;

    default:
      rv = -1;
  }
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












void ShowPrompt(void)
{
  void getMode(char *mode, u8 modeValue) {
    if (modeValue == 1)
      strncpy(mode, "demoOne", 8);
    else
      strncpy(mode, "led", 5);
  }
  char prompt[32] = "\r\n%s > ";
  char mode[16];
  getMode(&mode[0], *gDisplayMode);

  //strncpy(prompt, "\r\n%s> ", sizeof(prompt));
  //strncpy(frame, "Frame %d ", sizeof(frame));

  printf(prompt, mode);
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

  if (rv)
    rv = currentModeSerialCommandPlugin(cmd, args);

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

  updateAudioFilters();
  MX_ADC1_Init();
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
