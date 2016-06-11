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
  uint8_t red, green, blue;

  printf("#define COLOR_TABLE_LEN %d\n\n", PERIOD2);
  printf("ColorTypeDef ColorTable[%d] = {\n\n", PERIOD2);

  for(d = 0; d < PERIOD2; d++)
  {
    red   = (uint8_t)(128.0f + 120.0f*sin(w*d));
    green = (uint8_t)(128.0f + 120.0f*sin(w*(d+200)));
    blue  = (uint8_t)(128.0f + 120.0f*sin(w*(d+400)));
    printf("{0x%02x, 0x%02x, 0x%02x},\n", red, green, blue);
  }

  printf("\n};\n");
}
