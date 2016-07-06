// ControlRegisters.h
#ifndef __CONTROLREGISTERS_H
#define __CONTROLREGISTERS_H

#include "main.h"


// these offsets define a mapping for hw access via sw / communation protocol
#define DUMMY_READ                    0x0000
#define REVISION_OFFSET               0x0004
#define SCRATCHPAD_OFFSET             0x0008
#define ENVIRONSTATUS_OFFSET          0x0100 // : 0x020F
// reserved                           0x0210
#define DISPLAY_CONTROL_RESET         0x1000
#define DISPLAY_CONTROL_NLEDPCH       0x1001
#define DISPLAY_CONTROL_NCHAN         0x1002
#define DISPLAY_CONTROL_MAXCHAN       0x1003 // read-only
#define DISPLAY_STATUS_BR_OFFSET      0x1004
#define DISPLAY_STATUS_FD_OFFSET      0x1006
#define DISPLAY_STATUS_P_OFFSET       0x1008
#define DISPLAY_MODE_OFFSET           0x1010
#define DISPLAY_ERROR_OFFSET          0x1014 // clear-on-read + read-only
// reserved
#define BUFFER_STATUS_NFRAMES         0x2000
#define BUFFER_STATUS_CFRAME          0x2002
#define BUFFER_ADDRESS_OFFSET         0x2004
#define BUFFER_VALUE_OFFSET           0x2008
#define BUFFER_ERROR_OFFSET           0x200c // read-only
// reserved
#define DEMOn_CONTROL_OFFSET          0x3000

#define AUDIO_CONTROL_OFFSET          0x4000

//custom demo1 registers
#define DEMO1_T_VALUE                 0x000 // write only
#define DEMO1_T_INCR                  0x004
#define DEMO1_BRIGHTNESS              0x100
#define DEMO1_MODUPDATE               0x140
#define DEMO1_PERIOD                  0x180
#define DEMO1_PHASE                   0x1c0


#define DISPLAYMODE_RAW     0
#define DISPLAYMODE_REPEAT  9
#define DISPLAYMODE_DEMO1   1

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
buf FrameBuffer;




// this stores the actual data in RAM

struct ControlRegisters
{

  struct Revision // read only
  {
    u16 majorRev;
    u16 minorRev;
  } revision;

  u32 scratchPad;

  struct EnvironStatus // reserved, read-only
  {
    u32 temperature[16];
    u32 voltage[16];      // +0x0040
    u32 current[16];      // +0x0080
    u32 power[16];        // +0x00c0
    u8 faults[16];       //  +0x0100
  } environStatus;

  struct DisplayControl
  {
    u8 softReset;
    u8 numberLedsPerChannel;
    u8 maxNumberLedsPerChannel;
    u8 numberOfChannels;
    u8 maxNumberOfChannels; // read-only
    //struct MaxNumberOfStrips { struct RegSpec regSpec; u32 number; } maxNumberOfStrips;
  } displayControl;

  struct DisplayStatus
  {
    u16 brightness;
    u16 frameDelay;
    u8 pause;
  } displayStatus;

  u8 displayMode;
    // 0 -> raw
    // 1 -> repeat
    // >=2 -> demo_n

  u8 displayError; // clear-on-read, read-only
    // 0x1 -> invalid numberLedsPerStrip
    // 0x2 -> invalid numberOfStrips
    // 0x4 -> invalid brightness
    // 0x8 -> invalid frameRate
    // 0x10 -> unknown/invalid displayMode

  struct BufferStatus
  {
    u8 numberOfFrames;
    u8 currentFrame;
  } bufferStatus;

  struct BufferAddress
  {
    u8 frameId;
    u8 channelId;
    u8 pixelId;
    uint increment:1;
  } bufferAddress;

  Pixel bufferValue;

  u8 bufferError; // clear-on-read, read-only
    // 0x1 -> frame id out-of-range
    // 0x2 -> strip id out-of-range
    // 0x4 -> pixel id out-of-range
    // reserved
    // 0x80 -> last valid pixel written

  void *demoConfig;

  u8 accessStatus;

};

enum RegType
{
  reg8,
  reg16,
  reg32,
  regRevision,
  regDisplayControl,
  regDisplayStatus,
  regBufferStatus,
  regBufferAddress,
  regPixel,
  invalid,

} regTypes;

enum RegAction
{
  noRegAction,

  clear_on_read,

  funcSetBrightness,

  funcSetDemo1Brightness,
  funcSetDemo1Phase,
  funcSetDemo1Period,
  
  funcBufferAccess,
};

enum RegAccess
{
  rwRegAccess,
  read_only,
};

typedef struct {
  enum RegType type;
  enum RegAccess access;
  enum RegAction action;
  union _register
  {
    uint8_t  *reg8;
    uint16_t *reg16;
    uint32_t *reg32;
    struct Revision *revision;
    struct DisplayControl *displayControl;
    struct DisplayStatus *displayStatus;
    struct BufferStatus *bufferStatus;
    struct BufferAddress *bufferAddress;
    Pixel *pixel;
  } r;
} Register;





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
