/*
   FFT Audio Spectrum Analyzer
   Sampling frequency on Arduino Uno is approximately 9 KHz <--- nah we can do better
   Adapted from Gadget Reboot:              https://www.youtube.com/watch?v=djGOOL2mqV8
   Which was adapted from learnelectronics:      https://www.youtube.com/watch?v=5RmQJtE61zE
   Which was adapted from cbm80amiga:  https://www.youtube.com/watch?v=EnvhEgjrHsw

   Uses the FIX_FFT library:    https://www.arduinolibraries.info/libraries/fix_fft

   Gadget Reboot
*/

#include "better_fft.h"
#include "fix_fft.h"                                  // fixed point Fast Fourier Transform library
#include "avdweb_AnalogReadFast.h"                    // ~20us analog read
#include <Streaming.h>                                // debugging
#include "StopWatch.h"                                // debugging

#include <CircularBuffer.h>

#define audioIn A1                                    // audio input port

const int n = 256;
const int m = 8;
bool isBufferNotFull=true;
CircularBuffer<char,n> samples;
volatile int sample;
int mag[n/2];
char re[n], im[n];                                // real and imaginary FFT result arrays
static Stopwatch stopwatch(micros),stopwatch2(micros); 

void setup() {
  Serial.begin(2000000);
  timer1SampleInterrupt();
  analogReference(DEFAULT);                           // use the default analog reference of 5 volts (on 5V Arduino boards)
  pinMode(5,OUTPUT);                                                    // or 3.3 volts (on 3.3V Arduino boards)
    analogWrite(5,128); // squarewave,~900Hz for testing, not great for calibration

};

void loop() {
//  stopwatch.start(); 

  // The FFT real/imaginary data are stored in a char data type as a signed -128 to 127 number
  // This allows a waveform to swing centered around a 0 reference data point
  // The ADC returns data between 0-1023 so it is scaled to fit within a char by dividing by 4 and subtracting 128.
  // eg (0 / 4) - 128 = -128 and (1023 / 4) - 128 = 127
  
//  for (int i = 0; i < n; i++) {                    // read 128 analog input samples from ADC
////    sample = (analogReadFast(audioIn)/4)-128;
////    samples.push(sample);  
//    im[i]=0;
//  };
//  stopwatch.stop(); Serial << stopwatch.interval << "us to take " << n << " samples" << endl;

  // in-place FFT saves memory, but we don't do anything with the saved space from converting the 10 bit adc values to 8. If the mcu can handle it, let's bump this up.
//  fix_fft(re, im, m, 0);                              // send the samples for FFT conversion, returning the real/imaginary results in the same arrays
//   stopwatch2.start();

  cli();
  better_fft(samples,re,im,m,0);
  for(int i = 0;i<n;i++){
    Serial << (int)sqrt(re[i] * re[i] + im[i] * im[i]) << '\t';
  }
  sei();
  Serial << endl;
//    stopwatch2.stop(); Serial << stopwatch2.interval << ' ';
delay(2000);
  // The data array will contain frequency bin data in locations 0..255 for samples up to the sampling frequency of approx. 58 KHz
  // Each frequency bin will represent a center frequency of approximately (58 KHz / 256 samples) = 228 Hz
  // Due to Nyquist sampling requirements, we can only consider sampled frequency data up to (sampling rate / 2) = 29kHz
  // Therefore we only acknowledge the first 64 frequency bins [0..127] = [0..29KHz]

//Serial << (int)samples[255]+128;
//(unsigned)sample;
//Serial << stopwatch.interval;
//  for (int i = 0; i < n/2; i++) {
//    mag[i] = sqrt(re[i] * re[i] + im[i] * im[i]);     // frequency magnitude is the square root of the sum of the squares of the real and imaginary parts of a vector
//    Serial << (int)mag[i] << ' ';
//  };
Serial << endl; 
};

ISR(TIMER1_COMPA_vect){
//  stopwatch.interval=0;
//  stopwatch.start();
  sample=analogReadFast(audioIn);
  samples.push((sample/4)-128);
//  stopwatch.stop();
}

//http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
void timer1SampleInterrupt(){ // once every 25us = 40kHz~~~~ nope 20k for test

//// TIMER 1 for interrupt frequency 20000 Hz:
//cli(); // stop interrupts
//TCCR1A = 0; // set entire TCCR1A register to 0
//TCCR1B = 0; // same for TCCR1B
//TCNT1  = 0; // initialize counter value to 0
//// set compare match register for 20000 Hz increments
//OCR1A = 799; // = 16000000 / (1 * 20000) - 1 (must be <65536)
//// turn on CTC mode
//TCCR1B |= (1 << WGM12);
//// Set CS12, CS11 and CS10 bits for 1 prescaler
//TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
//// enable timer compare interrupt
//TIMSK1 |= (1 << OCIE1A);
//sei(); // allow interrupts

  
// TIMER 1 for interrupt frequency 40000 Hz:
cli(); // stop interrupts
TCCR1A = 0; // set entire TCCR1A register to 0
TCCR1B = 0; // same for TCCR1B
TCNT1  = 0; // initialize counter value to 0
// set compare match register for 40000 Hz increments
OCR1A = 399; // = 16000000 / (1 * 40000) - 1 (must be <65536)
// turn on CTC mode
TCCR1B |= (1 << WGM12);
// Set CS12, CS11 and CS10 bits for 1 prescaler
TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
// enable timer compare interrupt
TIMSK1 |= (1 << OCIE1A);
sei(); // allow interrupts
}
