// usart.c

#include "usart.h"
#include "gpio.h"

#include "stdarg.h"
#include "string.h"

UART_HandleTypeDef huart2;

char serialTxBuff[256];
char *serialTxBuff_p0 = &serialTxBuff[0];
char *serialTxBuff_p = &serialTxBuff[0];


#ifdef USE_PUTCHAR
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  *serialT     xBuff_p++ = (char)ch;
  HAL_UART_TxCpltCallback(&huart2);
  return ch;
}
#endif



char serialOutput[256];
volatile int __serialOutMutex = 0;

#ifndef USE_PUTCHAR
// add stuff to the serialTxBuffer until we can process it
int _write(int file, char *ptr, int len)
{
  int l;

  __serialOutMutex = 1;
  for(l = 0; l < len; l++)
  {
    *serialTxBuff_p = ptr[l];
    serialTxBuff_p++;
  }
  __serialOutMutex = 0;

  HAL_UART_TxCpltCallback(&huart2);

  return len;
}
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  int len;
  char *so, *si;

  if((len = serialTxBuff_p - serialTxBuff_p0))
  {
    // if we get callback happen during _write, pass on this and wait
    // for the next call from _write
    if (!__serialOutMutex)
    {
      so = serialOutput;
      si = serialTxBuff;
      __serialOutMutex = 1;
      strncpy(so, si, len);
      //for (l = 0; l < len; l++) { *so = *si; so++; si++; } // atomic copy to output
      __serialOutMutex = 0;
      if(HAL_UART_Transmit_IT(huart, (uint8_t*)serialOutput, len) == HAL_OK)
        serialTxBuff_p = serialTxBuff_p0; // reset tx buffer
    }
  }
}

char serialRxBuff[256];
char *serialRxBuff_p0 = &serialRxBuff[0];
char *serialRxBuff_p = &serialRxBuff[0];

volatile int __serialInMutex = 0;
/* add stuff to the serialRxBuffer until it's processed
int _read(int file, char *ptr, int len)
{
  HAL_UART_RxCpltCallback(&huart2);

  return len;
}*/

void sanitizePrintStr(char *str)
{
  while (*str != 0)
  {
    switch (*str)
    {
      case '\n':
        printf("\\n");
        break;
      case '\r':
        printf("\\r");
        break;
      default:
        if (iscntrl(*str)) printf("\\x%02x", *str);
        else printf("%c", *str);
    };
  str++;
  }
}

#define DebugSerialInput 1
void SerialInputHandler()
{
  int len;
  char *bs = "\x1b[D";
  static char lastChar;
  //static char lastLastChar;

  if ((len = serialRxBuff_p - serialRxBuff_p0) >= 255)
  {
    serialRxBuff[255] = '\0'; //coerce to null term string
    serialRxBuff_p = serialRxBuff_p0;
    printf("\r\nInvalid Data: \"");
    sanitizePrintStr(serialRxBuff);
    printf("\"\r\n");

    ShowPrompt();
  }

  else if (len)
  {
   // lastLastChar = lastChar;
    lastChar = *(serialRxBuff_p-1);

    switch (lastChar)
    {
      case '\r':
      case '\n':
        *serialRxBuff_p = '\0';
        serialRxBuff_p = serialRxBuff_p0;

        if (DebugSerialInput) {
          printf("Command Recieved: \"");
          sanitizePrintStr(serialRxBuff);
          printf("\"\r\n");
        }
        ParseCommands(serialRxBuff);

        ShowPrompt();
        break;
      case '\x7f': //backspace?
        _write(0, bs, 2);
        serialRxBuff_p-=2;
        *serialRxBuff_p = '\0';
        break;
      default:
        break;
    }


  }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static char serialRxChar;

  if (serialEchoEnabled) _write(0, &serialRxChar, 1);
   *serialRxBuff_p = serialRxChar; serialRxBuff_p++; 

  SerialInputHandler();

  while(
    HAL_UART_Receive_IT(huart, (uint8_t*)&serialRxChar, 1) == HAL_BUSY);
}

void USART2_Init(void)
{
  huart2.Instance          = USART2;
  
  huart2.Init.BaudRate     = 9600;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();

  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE|UART_IT_TC);

  HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void HAL_UART_MspInit(UART_HandleTypeDef* usartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(usartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    PA4     ------> USART2_CK 
    */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    */
  }

}