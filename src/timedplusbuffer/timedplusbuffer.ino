/*
   FFT Audio Spectrum Analyzer
   Sampling frequency on Arduino Uno is approximately 9 KHz <--- nah we can do better
   Adapted from Gadget Reboot:              https://www.youtube.com/watch?v=djGOOL2mqV8
   Which was adapted from learnelectronics:      https://www.youtube.com/watch?v=5RmQJtE61zE
   Which was adapted from cbm80amiga:  https://www.youtube.com/watch?v=EnvhEgjrHsw

   Uses the FIX_FFT library:    https://www.arduinolibraries.info/libraries/fix_fft

   Gadget Reboot
*/

// much more consistent than free-running adc
//20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 16 20 20 20 20 20 20 
//20 20 20 20 20 16 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 
//20 20 20 20 20 20 20 20 24 20 20 20 20 20 20 20 20 20 20 20 20 20 
//20 20 20 20 20 20 20 20 20 20 20 20 16 20 20 20 20 20 20 20 24 20 
//20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 12 20 20 20 
//20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 
//24 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20

#include "better_fft.h"
#include "fix_fft.h"                                  // fixed point Fast Fourier Transform library
//#include "avdweb_AnalogReadFast.h"                    // ~20us analog read
#include <Streaming.h>                                // debugging
#include "StopWatch.h"                                // debugging

#include <CircularBuffer.h>

#define audioIn A1                                    // audio input port

const int n = 256;
const int m = 8;
volatile bool fullBuffer,nextSample,samplingDone = false;
volatile byte bufferIndex=0x00;
bool isBufferNotFull=true;
CircularBuffer<char,n> samples;
volatile int sample;
int mag[n/2];
volatile char re[n], im[n];                                // real and imaginary FFT result arrays
static Stopwatch stopwatch(micros),stopwatch2(micros); 

void setup() {
  Serial.begin(2000000);
//  pinMode(13,OUTPUT);
//  timer1SampleInterrupt();
//  analogReference(DEFAULT);                           // use the default analog reference of 5 volts (on 5V Arduino boards)
  pinMode(5,OUTPUT);                                                    // or 3.3 volts (on 3.3V Arduino boards)
    analogWrite(5,128); // squarewave,~900Hz for testing, not great for calibration
//  pinMode(audioIn,INPUT);
initTimerOne();
  cli();//disable interrupts
  
  //set up continuous sampling of analog pin 0 at 38.5kHz
 
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0; // free running mode
//  ADCSRB |= 0b101; // use timer 1 compare match b as trigger
  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only
//  ADMUX &= ~(1 << ADLAR);//right align
  ADCSRA |= (1 << ADPS2) /*| (1 << ADPS1)*/ | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements
  
  sei();//enable interrupts
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
//

//Serial << (ADC>>6) << ' ';

//if(nextSample){
//  nextSample=false;
//delay(100);
//if(nextSample){
//  Serial << (int)bufferIndex << endl;
//  nextSample=false;
//  Serial << (int)re[bufferIndex] << ' ';
//}
  if(fullBuffer){
//    Serial << endl;
    cli();
//    for(int i = 0; i < n; i++){
//      Serial << (int)re[i] <<' ';
//    }
    fix_fft(re,im,m,0);
    for(int i = 0; i < n; i++){
    Serial << sqrt(re[i] * re[i] + im[i] * im[i])<<'\t';
    }
    Serial << endl;
//    bufferIndex=0;
    fullBuffer=false;
    TIFR1|= _BV(ICF1);
//    ADCSRA|=(1<<ADEN);// turn on adc
    ADCSRA|=(1<<ADIE); // turn on adc interrupt

    sei();
    
//    Serial << endl;

//    fullBuffer=true;
  } else {
    Serial << stopwatch.interval << '\n';
  }
  
//}

//if(fullBuffer){
//  
//  cli();
//  bufferIndex=0;
//  fix_fft(re,im,m,0);
////  better_fft(samples,re,im,m,0);
//  for(int i = 0;i<n;i++){
//    Serial << (float)100*sqrt(re[i] * re[i] + im[i] * im[i])<<' ';
//  }
//  sei();
//  Serial << endl;



//  Serial << stopwatch.maxInterval(1) << endl; // how long did the whole FFT and prints take?
//}
//delay(1000);
//Serial << stopwatch.interval << ' '; // longest time between ADC ISR calls
//  sei();
//  Serial << endl;
//    stopwatch2.stop(); Serial << stopwatch2.interval << ' ';
//delay(2000);
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
//Serial << endl; 
};

ISR(ADC_vect) { // called when new ADC value ready
if(fullBuffer){
  Serial << "This should not print because we disable adc interrupts" << endl;
  // TODO: disable adc interrupt bit here?
} else if(bufferIndex==0xFF){
  stopwatch.stop();
  fullBuffer=true;
  re[bufferIndex++]=ADCH-128;
//  ADCSRA&=~(1<<ADEN); // turn off adc
  ADCSRA&=~(1<<ADIE); // turn off adc interrupt but not adc
} else {
  stopwatch.stop();
  stopwatch.start();

//  TIFR1|= _BV(ICF1);
  re[bufferIndex++]=ADCH-128;
}
//  nextSample = true;
}


void initTimerOne(){
  // credit:6v6gt
  //https://forum.arduino.cc/index.php?topic=584634.0
  // Initialise Timer1
  TCCR1A = 0;
  TCCR1B = _BV(CS10) |     // Bit 2:0 - CS12:0: Clock Select =  no prescaler
      _BV(WGM13) |    // WGM 12 = CTC ICR1 Immediate MAX
      _BV(WGM12);     // WGM 12 ditto

  ICR1 = 99; // TOP for 40kHz

  // enable timer compare interrupt
  //TIMSK1 |= (1 << OCIE1B);
}

void initADC(){
    // Analog Port PC0 (pin A0 )
    // V0_02 ADLAR and prepare also for reading A1
    ADMUX = _BV(REFS0) | _BV(ADLAR) ;     // Fixed AVcc reference voltage for ATMega328P !! and left adjusted data for reading 8 bits if adlar is set
    DIDR0 |= _BV(ADC0D);             // DIDR0  Digital Input Disable Register 0
    DIDR0 |= _BV(ADC1D);             // DIDR0  Digital Input Disable Register 1
    ADCSRB =  _BV(ADTS2) |     // Bit 2:0  ADTS[2:0]: ADC Auto Trigger Source
        _BV(ADTS1) |     //   Timer/Counter1 Capture Event
        _BV(ADTS0);      //


  ADCSRA =    _BV(ADEN) |      // Bit 7   ADEN: ADC Enable
      _BV(ADSC) |      // Bit 6   ADSC: ADC Start Conversion
      _BV(ADATE) |     // Bit 5   ADATE: ADC Auto Trigger Enable
      _BV(ADIE) |      //
      _BV(ADPS1);      // Bits 2:0  ADPS[2:0]: ADC Prescaler Select Bits  (div 4 )  // V0_02 div2 NOK, div4 OK, div16 OK
}
