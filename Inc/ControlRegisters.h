// ControlRegisters.h
#ifndef __CONTROLREGISTERS_H
#define __CONTROLREGISTERS_H


#include "stdint.h"

#define u32 uint32_t
#define u16 uint16_t
#define u8  uint8_t
#define uint unsigned int

// FIXME redefining from main..
  #define MAJOR_REV 0
  #define MINOR_REV 1
  #define NUM_LEDS_PER_CHANNEL 60
  #define DEFAULT_BRIGHTNESS 50
  #define DEFAULT_FRAMERATE 10


// these offsets define a mapping for hw access via sw / communation protocol

#define REVISION_OFFSET               0x0000
#define SCRATCHPAD_OFFSET             0x0004
#define ENVIRONSTATUS_OFFSET          0x0100 // : 0x020F
// reserved                           0x0210
#define DISPLAYCONTROL_OFFSET         0x1000
#define DISPLAYSTATUS_OFFSET          0x1004
#define DISPLAYMODE_OFFSET            0x1008
#define DISPLAY_ERROR_OFFSET          0x100c
// reserved
#define BUFFERSTATUS_OFFSET           0x2000
#define BUFFERADDRESS_OFFSET          0x2004
#define BUFFERVALUE_OFFSET            0x2008
#define BUFFERERROR_OFFSET            0x200c

#define DEMOn_CONTROL_OFFSET          0x3000

//custom demo1 registers
#define DEMO1_BRIGHTNESS              0x0
#define DEMO1_FRAMEDELAY              0x4
#define DEMO1_PERIOD                  0x8
#define DEMO1_PHASE                   0xc


#define DISPLAYMODE_RAW     0
#define DISPLAYMODE_REPEAT  1
#define DISPLAYMODE_DEMO1   2

#define DISPLAYERROR_NLED   0x1
#define DISPLAYERROR_NCH    0x2
#define DISPLAYERROR_BR     0x4
#define DISPLAYERROR_FR     0x8
#define DISPLAYERROR_DM     0x10

#define BUFFER_ERROR_FRAMEID    0x1
#define BUFFER_ERROR_CHANNELID  0x2
#define BUFFER_ERROR_PIXELID    0x4
#define BUFFER_ERROR_LASTPIXEL  0x80

#define INVALID_ACCESS       1
#define UPDATE_VALID_ACCESS  0x80

#include "FrameBuffer.h"
buf frameBuffer;

void InitControlRegisters();

typedef enum handleRegAccessFlag
{
  validAccess,
  invalidAccess,
  clearOnReadAccess, // FIXME ???
  readOnlyAccess,
} HandleRegAccessFlag;


typedef struct _rawRegReadValue
{
  uint32_t reg;
  HandleRegAccessFlag flag;
} RawRegReadValue;

void RawRegisterRead(uint address, RawRegReadValue *rv);
HandleRegAccessFlag RawRegisterWrite(uint address, uint32_t value);

#endif