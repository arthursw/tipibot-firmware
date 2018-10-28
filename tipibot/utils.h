#include <stdio.h>

// Parse a float from a String (not the same in c++ in the simulator and in Arduino)
float stof(String& str) {
  #ifdef SIMULATOR
    return std::stof(str);
  #else
    return str.toFloat();
  #endif
}

// Convert from ortho to polar coordinates
void orthoToPolar(float x, float y, long* l, long* r) {
  float x2 = x * x;
  float y2 = y * y;
  float WmX = machineWidth - x;
  float WmX2 = WmX * WmX;
  *l = round(sqrt(x2 + y2) / millimetersPerStep);
  *r = round(sqrt(WmX2 + y2) / millimetersPerStep);
}

// Convert from polar to ortho coordinates
void polarToOrtho(long l, long r, float* x, float* y) {
  float lf = l * millimetersPerStep;
  float rf = r * millimetersPerStep;
  float l2 = lf * lf;
  float r2 = rf * rf;
  float w2 = machineWidth * machineWidth;
  float x2 = *x * *x;
  *x = (l2 - r2 + w2) / ( 2.0 * machineWidth );
  *y = sqrt(l2 - x2);
}
