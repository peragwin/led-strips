#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdint.h"

#define PI 3.1415926
#define PERIOD2 600

int main()
{
  const float w = PI/300;
  int d;
  float red_, green_, blue_, s;
  uint8_t red, green, blue;

  printf("#define COLOR_TABLE_LEN %d\n\n", PERIOD2);
  printf("Pixel ColorTable[%d] = {\n\n", PERIOD2);

  for(d = 0; d < PERIOD2; d++)
  {
    red_   = (128.0f + 120.0f*sin(w*d));
    green_ = (128.0f + 120.0f*sin(w*(d+200)));
    blue_  = (128.0f + 120.0f*sin(w*(d+400)));
    s = red_ + green_ + blue_;
    red = (uint8_t)(red_ * 255.0f/s);
    green = (uint8_t)(blue_ * 255.0f/s);
    blue = (uint8_t)(green_ * 255.0f/s);
    printf("{0x%02x, 0x%02x, 0x%02x}, // %f, %d \n", red, green, blue, s, (red+green+blue));
  }

  printf("\n};\n");


  char *foo = "foo ";
  char *f = foo;
  char *bar = "bar ";
  char *b = bar;
  printf("foo: %s\n", foo);
  for(int i=0; i < 3; i++){
    *++f = *b++;
    printf("foo: %s\n", foo);
  }

}
