// Audio Spectrum Display
// Copyright 2013 Tony DiCola (tony@tonydicola.com)

// This code is part of the guide at http://learn.adafruit.com/fft-fun-with-fourier-transforms/

#include <OctoWS2811.h>
#define ARM_MATH_CM4
#include <arm_math.h>
#include "Util.h"
#include "SerialCommand.h"

////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION 
// These values can be changed to alter the behavior of the spectrum display.
////////////////////////////////////////////////////////////////////////////////

const int FFT_SIZE = 256;
int SAMPLE_RATE_HZ = 9000;             // Sample rate of the audio in hertz.
float SPECTRUM_MIN_DB = 65.0;          // Audio intensity (in decibels) that maps to low LED brightness.
float SPECTRUM_MAX_DB = 83.0;          // Audio intensity (in decibels) that maps to high LED brightness.

const int AUDIO_INPUT_PIN = A14;        // Input ADC pin for audio data.
const int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
const int ANALOG_READ_AVERAGING = 16;  // Number of samples to average with each ADC reading.
const int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 3.0's onboard LED).

const int STRIP_LENGTH = 110;
const int BAR_LENGTH = STRIP_LENGTH;       // Number of neo pixels in BAR

////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////

DMAMEM int displayMemory[STRIP_LENGTH*6];
int drawingMemory[STRIP_LENGTH*6];

const int config = WS2811_GRB | WS2811_800kHz;

IntervalTimer samplingTimer;
float samples[FFT_SIZE*2];
float magnitudes[FFT_SIZE];
int sampleCounter = 0;

OctoWS2811 pixels(STRIP_LENGTH, displayMemory, drawingMemory, config);
SerialCommand sCmd;

float frequencyWindow[] = {2, 500, 5001, 5000, 5001, 16000};
float hues[BAR_LENGTH];
int history[BAR_LENGTH+1];

enum Mode { BeatController, Manual, AutoMood };
Mode mode = BeatController;

enum Mood { netflixAndChill, plasma, slowFade, restMode };
Mood mood = restMode;

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
  
  // Initialize neo pixel library and turn off the LEDs
  pixels.begin();
  pixels.show(); 
  
  // Initialize spectrum display
  spectrumSetup();
  
  // Begin sampling audio
  samplingBegin();
  
  // Turn on the power indicator LED.
  pinMode(POWER_LED_PIN, OUTPUT);
  digitalWrite(POWER_LED_PIN, HIGH);
  
  sCmd.addCommand("cGetMode", cGetMode);
  sCmd.addCommand("cBeatControllerOn", cBeatControllerOn);
  sCmd.addCommand("cSetPixelUniform", cSetPixelUniform);
  sCmd.addCommand("cSetPixelStrip", cSetPixelStrip);
  sCmd.addCommand("cSetPixelManual", cSetPixelManual);
  sCmd.addCommand("cSetStripColor", cSetStripColor);
  sCmd.addCommand("cSetUniformColor", cSetUniformColor);
  sCmd.addCommand("cShowPixels", cShowPixels);
  
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
  
    //see if we got some new commands
    sCmd.readSerial();
    
    switch(mode) {
      case BeatController: 
        beatController();
        break;
      case Manual:
        break;
    }
  
    // Restart audio sampling.
    samplingBegin();
  }
}

void spectrumSetup() {
  // Evenly spread hues across all pixels.
  for (int i = 0; i < BAR_LENGTH; ++i) {
    hues[i] = 360.0*(float(i)/float(BAR_LENGTH-1));
  }
}

void beatController() {  
  // Update each LED based on the intensity of the audio 
  // in the associated frequency window.
  float brightness, hue;
  int bass, height;
  
  float intensity, otherMean, low, mid, high;
  for (int i = 0; i < 3; ++i) {
      windowMean(magnitudes, 
             frequencyToBin(frequencyWindow[i]),
             frequencyToBin(frequencyWindow[i+1]),
             &intensity,
             &otherMean);
      switch (i) {
        case 0:
          low = intensity;
          break;
        case 1:
          mid = intensity;
          break;
        case 2:
          high = intensity;
          break;
        default:
          break;
      }
  }

  
  // Convert intensity to decibels.
  bass = 20.0*log10(low*2);
  
  height = map((int) bass, 63, 75, 0, BAR_LENGTH/2);
  height = constrain(height, 0, BAR_LENGTH/2);
  
  hue = map((int) bass, 64, 77, 0, 360);
  hue = constrain(hue, 0, 360);
  
  //end mids processing
  
  memmove(history, history+1, sizeof history - sizeof *history);
  brightness = map((int) bass, 66, 73, 0, 100);
  brightness = constrain(brightness, 0, 100)/100.0;

  history[BAR_LENGTH] = pixelHSVtoRGBColor(hue, 0.96, brightness);

  beatHit(height, hue);
  
  //gets too visually confusing when height is this much with histogram below.
  if (height < 80) {
    for (int i = 0; i < BAR_LENGTH; ++i) {
      if (pixels.getPixel(i) == 0) {
        setPixelUniform(i, history[i]);
      }
    }
  }

  pixels.show();
}


void beatHit(int height, int hue) {
  int half = BAR_LENGTH/2;
  for (int i = 0; i < half; ++i) {
    int x = map(i, 0, half, half, 0);
    if (i > height) {
      setPixelUniform(x, 0);
      setPixelUniform(i+half, 0);
    } else {
      setPixelUniform(x, pixelHSVtoRGBColor(hue, 1.0, 1.0));
      setPixelUniform(i+half, pixelHSVtoRGBColor(hue, 1.0, 1.0));
    }
  }
}

//Used to draw pixel at same position on all strips
void setPixelUniform(uint8_t pos, uint32_t c) {
  for(int i=0; i < 8; i++){
    pixels.setPixel(STRIP_LENGTH*i+pos, c);
  }
}

void setPixelStrip(uint8_t strip, uint8_t pos, uint32_t c) {
  pixels.setPixel(STRIP_LENGTH*strip+pos, c);
}

////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter+1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE*2) {
    samplingTimer.end();
  }
}

////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE*2;
}

// Compute the average magnitude of a target frequency window vs. all other frequencies.
void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
    *windowMean = 0;
    *otherMean = 0;
    // Notice the first magnitude bin is skipped because it represents the
    // average power of the signal.
    for (int i = 1; i < FFT_SIZE/2; ++i) {
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
