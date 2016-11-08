// usart.h

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "ctype.h"
#include "stm32f4xx_hal.h"

//#define USE_PUTCHAR
//extern int _write(int, char*, int);

extern void Error_Handler(void);
extern int ParseCommands(char*);
extern volatile int showFrameDebug;
extern volatile int serialEchoEnabled;

#define SERIAL_OUT_BUFFER_SIZE 256

//#define __USE_PUTCHAR 1
#ifdef __USE_PUTCHAR
void __io_putchar(unsigned);
extern int _write(int, char*, int);
#else
int _write(int, char*, int);
#endif

void USART2_Init(void);
void USART6_Init(void);
void HAL_UART_MspInit(UART_HandleTypeDef* usartHandle);
extern void ShowPrompt(void);

int SendSerialBufferIfNotEmpty(UART_HandleTypeDef*);

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */