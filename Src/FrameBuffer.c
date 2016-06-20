// FrameBuffer.c


#include "FrameBuffer.h"


#include "colors.out"

const Pixel CBlack      = {0x00, 0x00, 0x00};
const Pixel CRed        = {0xFF, 0x00, 0x00};
const Pixel CGreen      = {0x00, 0xFF, 0x00};
const Pixel CBlue       = {0x00, 0x00, 0xFF};
const Pixel CCyan       = {0x00, 0xFF, 0xFF};
const Pixel CMagenta    = {0xFF, 0x00, 0xFF};
const Pixel CYellow     = {0xFF, 0xFF, 0x00};
const Pixel CWhite      = {0xFF, 0xFF, 0xFF};

Pixel setPixelBrightnessF (Pixel p, float b)
{
  p.red = (b*p.red > 255) ? 0xFF : (uint8_t)(100*b*p.red/100);
  p.green = (b*p.green > 255) ? 0xFF : (uint8_t)(100*b*p.green/100);
  p.blue = (b*p.blue > 255) ? 0xFF : (uint8_t)(100*b*p.blue/100);
  return p;
}

Pixel setPixelBrightness(Pixel c, uint brightness)
{
  return (Pixel)
    { c.red * 100*brightness/10000,
      c.green * 100*brightness/10000,
      c.blue * 100*brightness/10000 };
}

Pixel AdjustPixelBrightness(Pixel c)
{
    int b; //, g;
   // static int subBrightCn;
  //  subBrightCn++;

    b = globalBrightness;
    Pixel p = (Pixel)
    { c.red * 100*b/10000,
      c.green * 100*b/10000,
      c.blue * 100*b/10000 };

    // g = globalSubBrightness;
    // if (g > 0 && g <= 8)
    //     if (!(subBrightCn % g))
    //         p = CBlack;
    // if (g < 0 && g >= -8)
    //     if (subBrightCn % g)
    //         p = CBlack;

    return p;
}

void FB_SetPixel(buf frameBuffer, uint8_t channel, uint index, Pixel c)
{
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    // clear
    frameBuffer[(index*24)+i]    &= ~(0x01<<channel);
    frameBuffer[(index*24)+8+i]  &= ~(0x01<<channel);
    frameBuffer[(index*24)+16+i] &= ~(0x01<<channel);

    // set
    frameBuffer[(index*24)+i]    |= ((c.green<<i) & 0x80) >> (7-channel);
    frameBuffer[(index*24)+8+i]  |= (  (c.red<<i) & 0x80) >> (7-channel);
    frameBuffer[(index*24)+16+i] |= ( (c.blue<<i) & 0x80) >> (7-channel);
  }
}

Pixel FB_GetPixel(buf fb, uint8_t channel, uint pixelId)
{
  uint8_t i;
  Pixel c = { 0, 0, 0 };
  for (i = 0; i < 8; i++)
  {
    c.green |= ((fb[24*pixelId+i] >> channel) & 1) << i;
    c.red   |= ((fb[24*pixelId+i+8] >> channel) & 1) << i;
    c.blue  |= ((fb[24*pixelId+i+16] >> channel) & 1) << i;
  }
  return c;
}

// @frameBuffer: pointer to output buffer
// @pixelBlock: a num_channels width array of Pixel
// @pixelIdx:   index of pixel per channel (0-num_leds_per_channel)
void FB_FastGetPixel(buf frameBuffer, PixelBlock pixelBlock, uint pixelIdx)
{
    int i, ch;
    uint8_t green, red, blue;
    uint8_t miniBuf[24];

    for (i=0; i<24; i++)
      miniBuf[i] = frameBuffer[24*pixelIdx+i];


    for (ch=0; ch<NUM_CHANNELS; ch++)
    {
      green = red = blue = 0;
      for(i=0; i<8; i++)
      {
        green |= ((miniBuf[i]>>ch) & 1) << (7-i);
        red   |= ((miniBuf[i+8]>>ch) & 1) << (7-i);
        blue  |= ((miniBuf[i+16]>>ch) & 1) << (7-i);
      }
      pixelBlock[ch].green = green;
      pixelBlock[ch].red = red;
      pixelBlock[ch].blue = blue;
    }
}

void FB_FastSetPixel(buf frameBuffer, PixelBlock pixelBlock, uint pixelIdx)
{
  int i, ch;
  uint8_t green, red, blue;
  uint8_t miniBuf[24] = {0};

  for (ch=0; ch<NUM_CHANNELS; ch++)
  {
    green = pixelBlock[ch].green;
    red   = pixelBlock[ch].red;
    blue  = pixelBlock[ch].blue;
    for (i=0; i<8; i++)
    {
     // miniBuf[i]    &= ~(1<<ch);
     // miniBuf[i+8]  &= ~(1<<ch);
     // miniBuf[i+16] &= ~(1<<ch);

      miniBuf[i]    |= ((green>>(7-i)) & 1) << ch;
      miniBuf[i+8]  |= ((red>>(7-i))   & 1) << ch;
      miniBuf[i+16] |= ((blue>>(7-i))  & 1) << ch;
    }
  }

  for (i=0; i<24; i++) frameBuffer[24*pixelIdx+i] = miniBuf[i];
}

void FB_ShiftSetPixel(buf fb, PixelBlock pixelBlock, uint idx)
{
  PixelBlock tempBlock;

  FB_FastGetPixel(fb, tempBlock, idx);
  FB_FastSetPixel(fb, pixelBlock, idx);

  for (int i=0; i<NUM_CHANNELS; i++) pixelBlock[i] = tempBlock[i];
}

