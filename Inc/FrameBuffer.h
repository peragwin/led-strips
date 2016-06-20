// FrameBuffer.h
#ifndef __FRAMEBUFFER_H
#define __FRAMEBUFFER_H

#include "stdint.h"

#define uint unsigned int
//extern _rawBuffer

typedef uint8_t* buf;

typedef struct {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} Pixel;

// FIXME redefining this from main..
	#define MAX_NUM_CHANNELS 8
	#define NUM_CHANNELS 8
typedef Pixel PixelBlock[MAX_NUM_CHANNELS];

#define COLOR_TABLE_LEN 600
Pixel ColorTable[COLOR_TABLE_LEN];

const Pixel CBlack;//      = {0x00, 0x00, 0x00};
const Pixel CRed;//        = {0xFF, 0x00, 0x00};
const Pixel CGreen ;//     = {0x00, 0xFF, 0x00};
const Pixel CBlue     ;//  = {0x00, 0x00, 0xFF};
const Pixel CCyan;//       = {0x00, 0xFF, 0xFF};
const Pixel CMagenta;//    = {0xFF, 0x00, 0xFF};
const Pixel CYellow;//     = {0xFF, 0xFF, 0x00};
const Pixel CWhite   ;//   = {0xFF, 0xFF, 0xFF};

Pixel setPixelBrightnessF (Pixel, float);
Pixel setPixelBrightness (Pixel, uint);

extern uint globalBrightness;
extern int globalSubBrightness;
Pixel AdjustPixelBrightness(Pixel);

void  FB_SetPixel(buf frameBuffer, uint8_t channel, uint index, Pixel c);
Pixel FB_GetPixel(buf fb, uint8_t channel, uint pixelId);
void FB_FastGetPixel(buf frameBuffer, PixelBlock pixelBlock, uint pixelIdx);
void FB_FastSetPixel(buf frameBuffer, PixelBlock pixelBlock, uint pixelIdx);
void FB_ShiftSetPixel(buf fb, PixelBlock pixelBlock, uint idx);

#endif





