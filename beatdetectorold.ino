// Audio Spectrum Display
// Copyright 2013 Tony DiCola (tony@tonydicola.com)

// This code is part of the guide at http://learn.adafruit.com/fft-fun-with-fourier-transforms/

#include <OctoWS2811.h>
#define ARM_MATH_CM4
#include <arm_math.h>
#include "Util.h"

////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION
// These values can be changed to alter the behavior of the spectrum display.
////////////////////////////////////////////////////////////////////////////////

const int FFT_SIZE = 256;
int SAMPLE_RATE_HZ = 12000;             // Sample rate of the audio in hertz.
float SPECTRUM_MIN_DB = 65.0;          // Audio intensity (in decibels) that maps to low LED brightness.
float SPECTRUM_MAX_DB = 81.5; //83.0          // Audio intensity (in decibels) that maps to high LED brightness.

const int AUDIO_INPUT_PIN = A14;        // Input ADC pin for audio data.
const int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
const int ANALOG_READ_AVERAGING = 16;  // Number of samples to average with each ADC reading.
const int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 3.0's onboard LED).

const int STRIP_LENGTH = 110;

struct freqBand {
  int low;
  int high;
};

const int bands = 1;
const struct freqBand freqBands[bands] = {
  {2, 500}
};

//fire
// The display size and color to use
const unsigned int width = STRIP_LENGTH;
const unsigned int height = 6;

////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////

DMAMEM int displayMemory[STRIP_LENGTH * 6];
int drawingMemory[STRIP_LENGTH * 6];

const int config = WS2811_GRB | WS2811_800kHz;

IntervalTimer samplingTimer;
float samples[FFT_SIZE * 2];
float magnitudes[FFT_SIZE];
int sampleCounter = 0;

OctoWS2811 pixels(STRIP_LENGTH, displayMemory, drawingMemory, config);

// Arrays for fire animation
unsigned char canvas[width * height];
extern const unsigned int fireColor[100];

int history[width + 1];

////////////////////////////////////////////////////////////////////////////////
// MAIN SKETCH FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Set up serial port.
  Serial.begin(115200);

  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);

  // Initialize neo pixel library and turn off the pixels
  pixels.begin();
  pixels.show();

  // Begin sampling audio
  samplingBegin();

  // Turn on the power indicator LED.
  pinMode(POWER_LED_PIN, OUTPUT);
  digitalWrite(POWER_LED_PIN, HIGH);

  Serial.print("started!");
}

void loop() {
  // Calculate FFT if a full sample is available.
  if (samplingIsDone()) {
    // Run FFT on sample data.
    arm_cfft_radix4_instance_f32 fft_inst;
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&fft_inst, samples);
    // Calculate magnitude of complex numbers output by the FFT.
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);

    animateFire();

    //animateBeatBar();

    // Restart audio sampling.
    samplingBegin();
  }
}

int timeSinceLastSound;
void animateFire() {
  // Update each LED based on the intensity of the audio
  // in the associated frequency window.
  float intensity, otherMean;
  float level[bands];
  for (int i = 0; i < bands; ++i) {
    windowMean(magnitudes,
               frequencyToBin(freqBands[i].low),
               frequencyToBin(freqBands[i].high),
               &intensity,
               &otherMean);
    level[i] = intensity;
  }
  int db, avg;
  int focus, heat, cool, fireSize;

  db = 20.0 * log10(level[0] * 2);
  db = constrain(db, 0, 100);
  avg = runningAverage(0.03, db);

  if (avg > 32) {
    timeSinceLastSound = 0;
  }

  if (timeSinceLastSound > 10000) {
    if (timeSinceLastSound > 20000) {
      cool = map(timeSinceLastSound, 20000, 30000, 100, 600);
      heat = map(timeSinceLastSound, 20000, 30000, 26, 19);
      focus = map(timeSinceLastSound, 20000, 30000, 7, 4);
    } else {
      cool = map(timeSinceLastSound, 10000, 15000, 1000, 100);
      heat = map(timeSinceLastSound, 10000, 15000, 10, 26);
      focus = map(timeSinceLastSound, 10000, 15000, 1, 7);
    }
    cool = constrain(cool, 100, 1000);
    heat = constrain(heat, 10, 26);
    focus = constrain(focus, 1, 7);
    if (timeSinceLastSound > 30000) timeSinceLastSound = 12000;
    delay(30);
  } else {
    fireSize = map(db, avg, avg + 9, 0, width);
    fireSize = constrain(fireSize, 0, width);

    Serial.printf("%2d, %2d  -  %3d -", avg, db, fireSize);

    for (int i = 0; i < width; i++) {
      if (i > (width - fireSize) / 2 && i < (width - fireSize) / 2 + fireSize) {
        Serial.print("0");
      } else {
        Serial.print(".");
      }
    }

    Serial.println("");

    cool = 26;
    focus = map(fireSize, 0, width, 13, 5);
    heat = (fireSize * fireSize) / 120;

    cool = constrain(cool, 0, 100);
    heat = constrain(heat, 0, 90);
    focus = constrain(focus, 0, 100);
  }

  // Step 1: move all data up one line
  memmove(canvas + width, canvas, width * (height - 1));
  memset(canvas, 0, width);

  // Step 2: draw random heatspots on bottom line
  int heatspots = heat;
  if (heatspots > width - 8) heatspots = width - 8;
  while (heatspots > 0) {
    int x = random(width - 2) + 1;
    if (canvas[x] == 0) {
      canvas[x] = 99;
      heatspots--;
    }
  }

  // Step 3: interpolate
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int c = canvas[y * width + x] * focus;
      int n = focus;
      if (x > 0) {
        c = c + canvas[y * width + (x - 1)];
        n = n + 1;
      }
      if (x < width - 1) {
        c = c + canvas[y * width + (x + 1)];
        n = n + 1;
      }
      if (y > 0) {
        c = c + canvas[(y - 1) * width + x];
        n = n + 1;
      }
      if (y < height - 1) {
        c = c + canvas[(y + 1) * width + x];
        n = n + 1;
      }
      c = (c + (n / 2)) / n;
      int coolThreshold = (random(1000) * cool) / 10000;
      if (c > coolThreshold) {
        c = c - coolThreshold;
      } else {
        c = 0;
      }
      canvas[y * width + x] = c;
    }
  }

  // Step 4: render canvas to pixels
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int c = canvas[((height - 1) - y) * width + x];
      pixels.setPixel(xy(x, y), fireColor[c]);
    }
    //test pixels.setPixel(xy(y, y), 0xFFFFFF);
  }
  pixels.show();
}

void animateBeatBar() {
  // Update each LED based on the intensity of the audio
  // in the associated frequency window.
  float intensity, otherMean;
  float level[bands];
  for (int i = 0; i < bands; ++i) {
    windowMean(magnitudes,
               frequencyToBin(freqBands[i].low),
               frequencyToBin(freqBands[i].high),
               &intensity,
               &otherMean);
    level[i] = intensity;
  }
  float brightness, historyBrightness, hue, db, avg;
  int barSize;

  db = 20.0 * log10(level[0] * 2);
  db = constrain(db, 0, 100);
  avg = runningAverage(0.03, db);

  barSize = map(db * 10, avg * 10, avg * 10 + 130, 0, width);
  barSize = constrain(barSize, 0, width);

  hue = map(db * 10, avg * 10 - 10, avg * 10 + 140, 0, 360);
  hue = constrain(hue, 0, 360);

  brightness = map(db * 10, avg * 10 - 50, avg * 10 + 110, 20, 100);
  brightness = constrain(brightness, 0, 100) / 100.0;

  if (barSize < 10) barSize = 0;

  Serial.printf("%2.2f, %2.2f  -  %3.0d -", avg, db, barSize);

  for (int i = 0; i < width; i++) {
    if (i > (width - barSize) / 2 && i < (width - barSize) / 2 + barSize) {
      Serial.print("0");
    } else {
      Serial.print(".");
    }
  }

  Serial.println("");

  memmove(history, history + 1, sizeof history - sizeof * history);
  historyBrightness = map(db * 10, avg * 10, avg * 10 + 130, 0, 70);
  historyBrightness = constrain(historyBrightness, 0, 100) / 100.0;
  history[width] = pixelHSVtoRGBColor(hue, 0.96, historyBrightness);

  for (int x = 0; x < width; x++) {
    if (x > (width - barSize) / 2 && x < (width - barSize) / 2 + barSize) {
      setPixelUniform(x, pixelHSVtoRGBColor(hue, 1.0, brightness));
    } else {
      setPixelUniform(x, 0);
    }
  }

  //gets too visually confusing when height is this much with histogram below.
  if (barSize < 80) {
    for (int p = 0; p < width; ++p) {
      if (pixels.getPixel(p) == 0) {
        setPixelUniform(p, history[p]);
      }
    }
  }

  pixels.show();
}

//Used to draw pixel at same position on all strips
void setPixelUniform(uint8_t pos, uint32_t c) {
  for (int i = 0; i < 8; i++) {
    pixels.setPixel(STRIP_LENGTH * i + pos, c);
  }
}

void setPixelStrip(uint8_t strip, uint8_t pos, uint32_t c) {
  pixels.setPixel(STRIP_LENGTH * strip + pos, c);
}

unsigned int xy(unsigned int x, unsigned int y) {
  return oldMap(y, 0, height-1, height-1, 0) * width + x;
}

long oldMap(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter + 1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE * 2) {
    samplingTimer.end();
  }
}

////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000 / SAMPLE_RATE_HZ);
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE * 2;
}

// Compute the average magnitude of a target frequency window vs. all other frequencies.
void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
  *windowMean = 0;
  *otherMean = 0;
  // Notice the first magnitude bin is skipped because it represents the
  // average power of the signal.
  for (int i = 1; i < FFT_SIZE / 2; ++i) {
    if (i >= lowBin && i <= highBin) {
      *windowMean += magnitudes[i];
    }
    else {
      *otherMean += magnitudes[i];
    }
  }
  *windowMean /= (highBin - lowBin) + 1;
  *otherMean /= (FFT_SIZE / 2 - (highBin - lowBin));
}

// Convert a frequency to the appropriate FFT bin it will fall within.
int frequencyToBin(float frequency) {
  float binFrequency = float(SAMPLE_RATE_HZ) / float(FFT_SIZE);
  return int(frequency / binFrequency);
}
