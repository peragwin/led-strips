// ControlRegisters.c

#include "ControlRegisters.h"


struct Revision // read only
  {
    //struct RegSpec regSpec;
    u16 majorRev;
    u16 minorRev;
  };


// this stores the actual data in RAM

struct ControlRegisters
{

  struct Revision // read only
    revision;

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
    //struct RegSpec regSpec;
    u8 softReset;
    u8 numberLedsPerChannel;
    u8 numberOfChannels;
    u8 maxNumberOfChannels; // read-only
    //struct MaxNumberOfStrips { struct RegSpec regSpec; u32 number; } maxNumberOfStrips;
  } displayControl;

  struct DisplayStatus
  {
    //struct RegSpec regSpec;
    u16 brightness;
    uint frameRate:15;
    uint paused:1;
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




extern buf DMA_IO_FrameBuffer;
struct ControlRegisters controlRegisters;

void InitControlRegisters()
{
	struct ControlRegisters *cr = &controlRegisters;

	cr->revision = (struct Revision) { MAJOR_REV, MINOR_REV }; // revision
	cr->scratchPad = 0; //scratch
	cr->environStatus = (struct EnvironStatus) {}; //env status

	cr->displayControl = 
			(struct DisplayControl) { 0, NUM_LEDS_PER_CHANNEL, NUM_CHANNELS, MAX_NUM_CHANNELS }; // display control
	cr->displayStatus =
			(struct DisplayStatus) { DEFAULT_BRIGHTNESS, DEFAULT_FRAMERATE, 0};                 // display status
	cr->displayMode = DISPLAYMODE_DEMO1;                                           // display mode
	cr->displayError = 0;                                                           // display error

	cr->bufferStatus = (struct BufferStatus) { 1, 1 };    // buffer status
	cr->bufferAddress = (struct BufferAddress) { 0, 0, 0, 1 };  // buffer address
	cr->bufferValue = CBlack;           // buffer value
	cr->bufferError = 0;           // buffer error

	frameBuffer = DMA_IO_FrameBuffer;
}



void readControlRegister(struct ControlRegisters *cr, Register *reg, uint32_t address)
{
	int invalidAccess = 0;
	// default
	reg->action = noRegAction;
	reg->access = rwRegAccess;

	if (address == REVISION_OFFSET){
		reg->type = regRevision;
		reg->r.revision = &cr->revision;
	}
	
	else if (address == SCRATCHPAD_OFFSET) {
		reg->type = reg32;
		reg->r.reg32 = &cr->scratchPad;
	}
	
	else if (address < ENVIRONSTATUS_OFFSET + 0x110)
	{
		uint32_t subaddr = address - 0x100;
		if (subaddr < 0x100 && (subaddr % 4 == 0)) {
			reg->type = reg32;
			reg->r.reg32 = (uint32_t*)((uint32_t)&cr->environStatus + subaddr);
		}

		else if (subaddr < 0x110) {
			reg->type = reg8;
			reg->r.reg8 = &cr->environStatus.faults[subaddr - 0x110];
		}
		else invalidAccess = 1;
	}
	
	else if (address <= DISPLAY_ERROR_OFFSET)
	{
		if (address == DISPLAYCONTROL_OFFSET) {
			reg->type = regDisplayControl;
			reg->r.displayControl = &cr->displayControl;
		}

		else if (address == DISPLAYSTATUS_OFFSET) {
			reg->type = regDisplayStatus;
			reg->r.displayStatus = &cr->displayStatus;
		}

		else if (address == DISPLAYMODE_OFFSET) {
			reg->type = reg8;
			reg->r.reg8 = &cr->displayMode;
		}
		else if (address == DISPLAY_ERROR_OFFSET) {
			reg->type = reg8;
			reg->action = clear_on_read;
			reg->r.reg8 = &cr->displayError;
		}

		else invalidAccess = 1;
	}

	else if (address <= BUFFERERROR_OFFSET)
	{
		if (address == BUFFERSTATUS_OFFSET) {
			reg->type = regBufferStatus;
			reg->r.bufferStatus = &cr->bufferStatus;
		}
	
		else if (address == BUFFERADDRESS_OFFSET) {
			reg->type = regBufferAddress;
			reg->r.bufferAddress = &cr->bufferAddress;
		}
	
		else if (address == BUFFERVALUE_OFFSET) {
			reg->type = regPixel;
			reg->action = funcBufferAccess;
			reg->r.pixel = &cr->bufferValue;
		}
	
		else if (address == BUFFERERROR_OFFSET) {
			reg->type = reg8;
			reg->action = clear_on_read;
			reg->r.reg8 = &cr->bufferError;
		}
	
		else invalidAccess = 1;
	}

	else invalidAccess = 1;

	if (invalidAccess) {
		cr->accessStatus |= INVALID_ACCESS;
		reg->type = invalid;
		return;
	}
	else
	{
		cr->accessStatus |= UPDATE_VALID_ACCESS;
		return;
	}
}


HandleRegAccessFlag handleRegReadAction(struct ControlRegisters *cr, Register reg)
{
	int valid = 1;

	if (reg.action == clear_on_read)
		if (reg.type == reg8) // only valid case for now
			return clearOnReadAccess;

	/* begin buffer access handler */
	if (reg.action == funcBufferAccess)
	{ 
		uint8_t pixelId = cr->bufferAddress.pixelId;
		uint8_t chanId = cr->bufferAddress.channelId;
		uint8_t frameId = cr->bufferAddress.frameId;
		
		uint8_t maxLed = cr->displayControl.numberLedsPerChannel;
		uint8_t maxCh  = cr->displayControl.numberOfChannels;
		uint8_t maxFrame = cr->bufferStatus.numberOfFrames;

		if (cr->bufferAddress.increment)
		{
			pixelId++;

			if (pixelId == maxLed)
			{
				pixelId = 0;
				chanId++;
			}
			if (chanId == maxCh)
			{
				chanId = 0;
				frameId++;
			}
			if (frameId == maxFrame)
			{
				// flag last valid pixel written
				cr->bufferError |= BUFFER_ERROR_LASTPIXEL;
				frameId = 0;
			}

			cr->bufferAddress.pixelId = pixelId;
			cr->bufferAddress.channelId = chanId;
			cr->bufferAddress.frameId = frameId;
		}

		if (pixelId >= maxLed) {
			valid = 0;
			cr->bufferError |= BUFFER_ERROR_PIXELID;
		}

		if (chanId >= maxCh) {
			valid = 0;
			cr->bufferError |= BUFFER_ERROR_CHANNELID;
		}

		if (frameId >= maxFrame) {
			valid = 0;
			cr->bufferError |= BUFFER_ERROR_FRAMEID;
		}

		if (valid)
			// actually grab data from the buffer
			*reg.r.pixel = FB_GetPixel(frameBuffer, chanId, pixelId);
	}
	/* end buffer access handler */

	return valid ? validAccess : invalidAccess;
}



HandleRegAccessFlag handleRegWriteAction(struct ControlRegisters *cr, Register reg)
{
	int valid = 1;

	if (reg.access == read_only)
		return readOnlyAccess;

	/* begin buffer access handler */
	if (reg.action == funcBufferAccess)
	{ 
		uint8_t pixelId = cr->bufferAddress.pixelId;
		uint8_t chanId = cr->bufferAddress.channelId;
		uint8_t frameId = cr->bufferAddress.frameId;
		
		uint8_t maxLed = cr->displayControl.numberLedsPerChannel;
		uint8_t maxCh  = cr->displayControl.numberOfChannels;
		uint8_t maxFrame = cr->bufferStatus.numberOfFrames;

		if (pixelId >= maxLed) {
			valid = 0;
			cr->bufferError |= BUFFER_ERROR_PIXELID;
		}

		if (chanId >= maxCh) {
			valid = 0;
			cr->bufferError |= BUFFER_ERROR_CHANNELID;
		}

		if (frameId >= maxFrame) {
			valid = 0;
			cr->bufferError |= BUFFER_ERROR_FRAMEID;
		}

		if (valid)
			// actually set data in the buffer
			FB_SetPixel(frameBuffer, chanId, pixelId, *reg.r.pixel);
		else
			return invalidAccess;

		if (cr->bufferAddress.increment)
		{
			pixelId++;

			if (pixelId == maxLed)
			{
				pixelId = 0;
				chanId++;
			}
			if (chanId == maxCh)
			{
				chanId = 0;
				frameId++;
			}
			if (frameId == maxFrame)
			{
				// flag last valid pixel written
				cr->bufferError |= BUFFER_ERROR_LASTPIXEL;
				frameId = 0;
			}

			cr->bufferAddress.pixelId = pixelId;
			cr->bufferAddress.channelId = chanId;
			cr->bufferAddress.frameId = frameId;
		}
	}
	/* end buffer access handler */

	if (reg.action == funcSetDemo1Brightness);

	return valid ? validAccess : invalidAccess;
}


void RawRegisterRead(uint address, RawRegReadValue *rv)
{	
	Register reg = {};
	readControlRegister(&controlRegisters, &reg, address);
	
	rv->flag = handleRegReadAction(&controlRegisters, reg);
	
	if (rv->flag == invalidAccess)
		rv->reg = 0;
	
	else // valid or clearOnRead
		switch (reg.type)
		{
			case reg8: rv->reg = (uint32_t) *reg.r.reg8; break;
			case reg16: rv->reg = (uint32_t) *reg.r.reg16; break;
			case reg32: rv->reg = *reg.r.reg32; break;
			case regRevision: rv->reg = *(uint32_t*)reg.r.revision; break;
			case regDisplayControl: rv->reg = *(uint32_t*)reg.r.displayControl; break;
			case regDisplayStatus: rv->reg = *(uint32_t*)reg.r.displayStatus; break;
			case regBufferStatus: rv->reg = *(uint32_t*)reg.r.bufferStatus & 0x0000ffFF; break;
			case regBufferAddress: rv->reg = *(uint32_t*)reg.r.bufferAddress; break;
			case regPixel: rv->reg = *(uint32_t*)reg.r.pixel & 0x00ffFFff; break;
			case invalid:
			default:
				rv->reg = 0; break;
		}

	if (rv->flag == clearOnReadAccess)
		*reg.r.reg8 = 0; // only have this type now, so no need for switch on type

	return;
}


HandleRegAccessFlag RawRegisterWrite(uint address, uint32_t value)
{
	Register reg = {};
	readControlRegister(&controlRegisters, &reg, address);

	if (reg.access != read_only)
		switch (reg.type)
		{
			case reg8: *reg.r.reg8 = (uint8_t) value; break;
			case reg16: *reg.r.reg16 = (uint16_t) value; break;
			case reg32: *reg.r.reg32 = value; break;
			case regRevision: *(uint32_t*)reg.r.revision = value; break;
			case regDisplayControl: *(uint32_t*)reg.r.displayControl = value; break;
			case regDisplayStatus: *(uint32_t*)reg.r.displayStatus = value; break;
			case regBufferStatus:
					*(uint16_t*)reg.r.bufferStatus = value;
					break;
			case regBufferAddress: *(uint32_t*)reg.r.bufferAddress = value; break;
			case regPixel:
					*(uint32_t*)reg.r.pixel &= ~0x00ffFFff;
					*(uint32_t*)reg.r.pixel |= (value & 0x00ffFFff);
					break;
			case invalid: return invalidAccess;
			default: return invalidAccess;
		}

	return handleRegWriteAction(&controlRegisters, reg);
}