#include "Util.h"

// Convert from HSV values (in floating point 0 to 1.0) to RGB colors usable
// by neo pixel functions.
uint32_t pixelHSVtoRGBColor(float hue, float saturation, float value) {
  // Implemented from algorithm at http://en.wikipedia.org/wiki/HSL_and_HSV#From_HSV
  float chroma = value * saturation;
  float h1 = float(hue) / 60.0;
  float x = chroma * (1.0 - fabs(fmod(h1, 2.0) - 1.0));
  float r = 0;
  float g = 0;
  float b = 0;
  if (h1 < 1.0) {
    r = chroma;
    g = x;
  }
  else if (h1 < 2.0) {
    r = x;
    g = chroma;
  }
  else if (h1 < 3.0) {
    g = chroma;
    b = x;
  }
  else if (h1 < 4.0) {
    g = x;
    b = chroma;
  }
  else if (h1 < 5.0) {
    r = x;
    b = chroma;
  }
  else // h1 <= 6.0
  {
    r = chroma;
    b = x;
  }
  float m = value - chroma;
  r += m;
  g += m;
  b += m;
  return Color(int(255 * r), int(255 * g), int(255 * b));
}

// Returns the Red component of a 32-bit color
uint8_t Red(uint32_t color)
{
  return (color >> 16) & 0xFF;
}

// Returns the Green component of a 32-bit color
uint8_t Green(uint32_t color)
{
  return (color >> 8) & 0xFF;
}

// Returns the Blue component of a 32-bit color
uint8_t Blue(uint32_t color)
{
  return color & 0xFF;
}

int getMax(float* array, int size) {
  int maxIndex = 0;
  int max = array[maxIndex];
  for (int i = 1; i < size; i++) {
    if (max < array[i]) {
      max = array[i];
      maxIndex = i;
    }
  }
  return array[maxIndex];
}

uint32_t Color(byte r, byte g, byte b) {
  r = pgm_read_byte(&gammatable[r]);
  g = pgm_read_byte(&gammatable[g]);
  b = pgm_read_byte(&gammatable[b]);
  return r << 16 | g << 8 | b;
}

float runningAverage(float alpha, int M) {
  static float accum = 0;
  
  accum = (alpha * M) + (1.0 - alpha) * accum;

  return accum;
}

int fps() {
  // Create static variables so that the code and variables can
  // all be declared inside a function
  static unsigned long lastMillis;
  static unsigned long frameCount;
  static unsigned int framesPerSecond;

  // It is best if we declare millis() only once
  unsigned long now = millis();
  frameCount ++;
  if (now - lastMillis >= 5000) {
    framesPerSecond = frameCount / 5;
    Serial.printf("fps: %4d", framesPerSecond);
    frameCount = 0;
    lastMillis = now;
  }
  return framesPerSecond;
}
