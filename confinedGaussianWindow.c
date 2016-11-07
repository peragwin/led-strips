#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#define FFT_LENGTH 128
#define G_SIGMA 0.1

float wGaussian(float a){
  float aa = ( a - ((FFT_LENGTH-1)/2.0) )/( 2.0*G_SIGMA*FFT_LENGTH );
 //float printf("aa: %.4f, exp: %.4f\n", aa, exp( -(aa*aa)));
  return exp(  -(aa*aa) );
}

#define G_SIGMA 0.1
float confinedGaussian(float n) {
  float g1 = wGaussian(-0.5);
  float g2 = wGaussian(n + FFT_LENGTH) + wGaussian(n - FFT_LENGTH);
  float g4 = wGaussian(-0.5 + FFT_LENGTH) + wGaussian(-0.5 - FFT_LENGTH);
  return wGaussian(n) - g1 * g2 / g4;
}

float fftWindow[FFT_LENGTH];

int main(int argn, char **argv){
    for (int n = 0; n < FFT_LENGTH; n++)
        printf("%3d %.4f\n", n, confinedGaussian((float)n));

    return 0;
}