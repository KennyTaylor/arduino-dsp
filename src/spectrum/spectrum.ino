/*
   FFT Audio Spectrum Analyzer
   Sampling frequency on Arduino Uno is approximately 9 KHz <--- nah we can do better
   Adapted from Gadget Reboot:              https://www.youtube.com/watch?v=djGOOL2mqV8
   Which was adapted from learnelectronics:      https://www.youtube.com/watch?v=5RmQJtE61zE
   Which was adapted from cbm80amiga:  https://www.youtube.com/watch?v=EnvhEgjrHsw

   Uses the FIX_FFT library:    https://www.arduinolibraries.info/libraries/fix_fft

   Gadget Reboot
*/

#include "fix_fft.h"                                  // fixed point Fast Fourier Transform library
#include "avdweb_AnalogReadFast.h"                    // ~20us analog read
#include <Streaming.h>                                // debugging
#include "StopWatch.h"                                // debugging

#define audioIn A1                                    // audio input port

const int n = 256;
const int m = 8;

int sample;
int mag[n/2];
char re[n], im[n];                                // real and imaginary FFT result arrays
//static Stopwatch stopwatch(micros); 

void setup() {
  Serial.begin(2000000);
  analogReference(DEFAULT);                           // use the default analog reference of 5 volts (on 5V Arduino boards)
  pinMode(5,OUTPUT);                                                    // or 3.3 volts (on 3.3V Arduino boards)
  analogWrite(5,127); // squarewave,~900Hz for testing, not great for calibration

};

void loop() {
//  stopwatch.start(); 

  // The FFT real/imaginary data are stored in a char data type as a signed -128 to 127 number
  // This allows a waveform to swing centered around a 0 reference data point
  // The ADC returns data between 0-1023 so it is scaled to fit within a char by dividing by 4 and subtracting 128.
  // eg (0 / 4) - 128 = -128 and (1023 / 4) - 128 = 127

  for (int i = 0; i < n; i++) {                    // read 128 analog input samples from ADC
    sample = analogReadFast(audioIn);
    re[i] = sample / 4 - 128;                         // scale the samples to fit within a char variable
    im[i] = 0;                                        // there are no imaginary samples associated with the time domain so set to 0
  };
//  for(int i = 0; i < n; i++){
//        Serial << (int)re[i]<<' ';
//  }
//  stopwatch.stop(); Serial << stopwatch.interval << "us to take " << n << " samples" << endl;

  // in-place FFT saves memory, but we don't do anything with the saved space from converting the 10 bit adc values to 8. If the mcu can handle it, let's bump this up.
  fix_fft(re, im, m, 0);                              // send the samples for FFT conversion, returning the real/imaginary results in the same arrays

  // The data array will contain frequency bin data in locations 0..255 for samples up to the sampling frequency of approx. 58 KHz
  // Each frequency bin will represent a center frequency of approximately (58 KHz / 256 samples) = 228 Hz
  // Due to Nyquist sampling requirements, we can only consider sampled frequency data up to (sampling rate / 2) = 29kHz
  // Therefore we only acknowledge the first 64 frequency bins [0..127] = [0..29KHz]

  for (int i = 0; i < n/2; i++) {
    mag[i] = (unsigned)sqrt(re[i] * re[i] + im[i] * im[i]);     // frequency magnitude is the square root of the sum of the squares of the real and imaginary parts of a vector
//    Serial << mag[i] << ' ';
  };
Serial << endl;
};
