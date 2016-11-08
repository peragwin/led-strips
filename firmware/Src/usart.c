// usart.c

#include "usart.h"
#include "gpio.h"

//#include "stdarg.h"

#include "main.h"

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

int currentUartIrqPriority = 7;

char serialTxBuff[SERIAL_OUT_BUFFER_SIZE];
char *serialTxBuff_p0 = &serialTxBuff[0];
char *serialTxBuff_p = &serialTxBuff[0];

UART_HandleTypeDef *uaReplyChan;



void pushUartIrqPriority(void) {
  if (currentUartIrqPriority > 3) currentUartIrqPriority--;
  int irqChan = (uaReplyChan->Instance == USART2) ? USART2_IRQn : USART6_IRQn;
  HAL_NVIC_SetPriority(irqChan, currentUartIrqPriority, 1);
}

void popUartIrqPriority(void) {
  if (currentUartIrqPriority < 7) currentUartIrqPriority++;
  int irqChan = (uaReplyChan->Instance == USART2) ? USART2_IRQn : USART6_IRQn;
  HAL_NVIC_SetPriority(irqChan, currentUartIrqPriority, 1);
}






#ifdef __USE_PUTCHAR

void __io_putchar(unsigned c) {
  if (serialTxBuff_p >= serialTxBuff_p0+SERIAL_OUT_BUFFER_SIZE) { // overflow, have to become blocking
    HAL_UART_Transmit(uaReplyChan, serialTxBuff, SERIAL_OUT_BUFFER_SIZE, 1000);
  }
  *serialTxBuff_p = c;
  serialTxBuff_p++;

  SendSerialBufferIfNotEmpty(uaReplyChan);
}
/* defined in syscalls.c
int _write(int file, char *ptr, int len) {
  int l;
  for (l = 0; l < len; l++) {
    putchar(*ptr);
    ptr++;
  }
  return len;
}
*/
#else
// add stuff to the serialTxBuffer until we can process it
int _write(int file, char *ptr, int len)
{
  int l;
  int wr = 0;

  for(l = 0; l < len; l++)
  {
    *serialTxBuff_p = ptr[l];
    serialTxBuff_p++;
    if (serialTxBuff_p >= serialTxBuff+SERIAL_OUT_BUFFER_SIZE) {
      wr += SendSerialBufferIfNotEmpty(uaReplyChan);
    }
  }
  return wr + SendSerialBufferIfNotEmpty(uaReplyChan);
}
#endif


//char serialOutput[256];
//volatile 
int __serialOutMutex = 0;


int SendSerialBufferIfNotEmpty(UART_HandleTypeDef *huart) {
  int len;
  char *serialTxBuff_p_notok;

  if (huart == uaReplyChan) {
    if((len = serialTxBuff_p - serialTxBuff_p0)) {

      if (1) { //len >= SERIAL_OUT_BUFFER_SIZE) {
        if (HAL_UART_Transmit(huart, (uint8_t*)serialTxBuff, len, 1000) == HAL_OK){
          serialTxBuff_p = serialTxBuff_p0;
          return len;
        }
      }

      // buffer not full so we can be non-blocking
      else if (!__serialOutMutex) { 
        __serialOutMutex = 1;
        serialTxBuff_p_notok = serialTxBuff_p;
        serialTxBuff_p = serialTxBuff_p0;

        if (currentUartIrqPriority > 2) {
          pushUartIrqPriority();
        
          if(HAL_UART_Transmit_IT(huart, (uint8_t*)serialTxBuff, len) != HAL_OK) {
            serialTxBuff_p = serialTxBuff_p_notok; // reset tx buffer
            popUartIrqPriority();
            return 0;
          }
        }

        return len;
      }
    }
    else return 0;
  }

  return -1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  __serialOutMutex = 0;
  popUartIrqPriority();
  SendSerialBufferIfNotEmpty(huart);
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
  int len, rv;
  char *bs = "\x1b[D\x1b[3~";
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
        if (*(serialRxBuff_p-1) == '\r')
          serialRxBuff_p -= 1;
        *serialRxBuff_p = '\0';
        serialRxBuff_p = serialRxBuff_p0;

        if (DebugSerialInput) {
          printf("Command Recieved: \"");
          sanitizePrintStr(serialRxBuff);
          printf("\"\r\n");
        }
        rv = ParseCommands(serialRxBuff);
        if (rv)
          printf("command execution failed!\r\n");

        ShowPrompt();
        break;
      case '\x7f': //backspace?
        _write(0, bs, 7);
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
  static uint8_t serialRxChar;
  *serialRxBuff_p = (char)serialRxChar; serialRxBuff_p++; 

  if (serialEchoEnabled) { //_write(0, &serialRxChar, 1);
    pushUartIrqPriority();
    HAL_UART_Transmit(huart, &serialRxChar, 1, 100);
  }

  // ????? (not sure about this)
  // since these are on same irq channel, we can't process any new
  // transmit irqs until this iterrupt vector completes
  __serialOutMutex = 1;

  uaReplyChan = huart;
  SerialInputHandler();

  while (HAL_UART_Receive_IT(huart, &serialRxChar, 1) == HAL_BUSY)
    HAL_Delay(20);

  __serialOutMutex = 0;
}

void USART2_Init(void)
{
  huart2.Instance          = USART2;
  
  huart2.Init.BaudRate     = 115200;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();

  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE|UART_IT_TC);

  uaReplyChan = &huart2;
  printf("Led Controller v%d.%d\r\n", MAJOR_REV, MINOR_REV);
  ShowPrompt();
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
  
  if(usartHandle->Instance==USART6)
  {
    __HAL_RCC_USART6_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA11     ------> USART6_TX
    PA12     ------> USART6_RX

    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  }

}

void USART6_Init(void)
{
  huart6.Instance          = USART6;
  
  huart6.Init.BaudRate     = 115200;
  huart6.Init.WordLength   = UART_WORDLENGTH_8B;
  huart6.Init.StopBits     = UART_STOPBITS_1;
  huart6.Init.Parity       = UART_PARITY_NONE;
  huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart6.Init.Mode         = UART_MODE_TX_RX;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_Init(&huart6) != HAL_OK) Error_Handler();

  __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE|UART_IT_TC);

  uaReplyChan = &huart6;
  printf("Led Controller v%d.%d\r\n", MAJOR_REV, MINOR_REV);
  ShowPrompt();
}
