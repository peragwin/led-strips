// main.h

#define u32 uint32_t
#define u16 uint16_t
#define u8  uint8_t
//#define uint unsigned int


#define PRINT_DEBUG


#define MAJOR_REV 0
#define MINOR_REV 8

// buffer size is num_leds * 24 bits/led
// 60 * 24 = 1440
#define BUFFSIZE 2*1440
#define NUM_CHANNELS 8
#define NUM_LEDS_PER_CHANNEL 2*60
#define MAX_NUM_LEDS_PER_CHANNEL 240
#define MAX_NUM_CHANNELS 8

#define ADC_BUFFER_LENGTH 512

#define DEFAULT_BRIGHTNESS 100
#define DEFAULT_FRAMEDELAY 100

#define min(X, Y)  ((X) < (Y) ? (X) : (Y))

#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "stdint.h"

#include "stm32f4xx_hal.h"

//#define ARM_MATH_CM4
#include "arm_math.h"
  #include "math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"