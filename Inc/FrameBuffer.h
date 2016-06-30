// FrameBuffer.h
#ifndef __FRAMEBUFFER_H
#define __FRAMEBUFFER_H

#include "main.h"


typedef uint8_t* buf;

typedef struct {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} Pixel;
typedef Pixel PixelBlock[MAX_NUM_CHANNELS];

#define CBlack         (Pixel) {0x00, 0x00, 0x00}
#define CRed           (Pixel) {0xFF, 0x00, 0x00}
#define CGreen         (Pixel) {0x00, 0xFF, 0x00}
#define CBlue          (Pixel) {0x00, 0x00, 0xFF}
#define CCyan          (Pixel) {0x00, 0xFF, 0xFF}
#define CMagenta       (Pixel) {0xFF, 0x00, 0xFF}
#define CYellow        (Pixel) {0xFF, 0xFF, 0x00}
#define CWhite         (Pixel) {0xFF, 0xFF, 0xFF}

Pixel setPixelBrightnessF (Pixel, float);
Pixel setPixelBrightness (Pixel, uint);

extern uint16_t *globalBrightness;
extern int globalSubBrightness;
Pixel AdjustPixelBrightness(Pixel, uint);

void  FB_SetPixel(buf frameBuffer, uint8_t channel, uint index, Pixel c);
Pixel FB_GetPixel(buf fb, uint8_t channel, uint pixelId);
void FB_FastGetPixel(buf frameBuffer, PixelBlock pixelBlock, uint pixelIdx);
void FB_FastSetPixel(buf frameBuffer, PixelBlock pixelBlock, uint pixelIdx);
void FB_ShiftSetPixel(buf fb, PixelBlock pixelBlock, uint idx);

#endif





