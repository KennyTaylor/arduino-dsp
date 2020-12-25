// real-time fft and histogram on oled, 128px bins x 64px magnitude resolution
// TODO: hardware timing tests, more precise than micros()

#include "fix_fft.h"

////////////////////////////OLED STUFF
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels // scale this down if more space is needed

// Declaration for SSD1306 display connected using software SPI (default case):
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
                         OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
////////////////////////////OLED STUFF

#define n 128
#define m 7

//const byte PROGMEM n = 128;
//const byte PROGMEM m = 7;
volatile char samples[2*n];//, im[n];
const char PROGMEM im[n]; // pita;
volatile byte bufferIndex = 0;
volatile bool bufferFull = true;
volatile bool secondSampleSet = false;

void setup() {
  pinMode(7,INPUT_PULLUP);
  pinMode(6,INPUT_PULLUP);
  pinMode(5, OUTPUT); // testing
  analogWrite(5, 128);
  Serial.begin(2000000);
  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  cli();
  initTimerOne();
  initADC();
  sei();
//  display.display();
//  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0, 0);            // Start at top-left corner
  display.clearDisplay();
  display.dim(true);
}

void loop() {
  if(digitalRead(7)==LOW){
//      display.display();
    while(digitalRead(6)==HIGH){
      delay(100);
    }
  }
  if (bufferFull) {
    display.clearDisplay();
//    cli();
    if(secondSampleSet){
      cli();
//      //print
//      Serial.println();
//      Serial.println();
//      for (byte i = 128; i > 0; i++) { // saving a couple bits, don't get confused by it
//        Serial.print((int)(samples[i]));
//        Serial.print(' ');
//        }
//        Serial.println();
      fix_fftr(&samples[128], m, 0);
//      fix_fft(&samples[128], im, m, 0); // second sample fft
//      sei();      
      for(byte j=0;j<126;j++){ // low pass filter
        samples[255-j]=-127;
      }
//      for(byte j=0;j<100;j++){ // high pass filter
//        samples[128+j]=127;
//      }
      fix_fftr(&samples[128], m, 1);
//      fix_fft(&samples[128], im, m, 1); // second sample inverse fft
      for (byte i = 128; i > 0; i+=2) { // saving a couple bits, don't get confused by it
//        Serial.print((int)(samples[i]));
//        Serial.print(' ');
//        display.drawLine(i-128, SCREEN_HEIGHT/2, i-128, (SCREEN_HEIGHT/2)-
//        (
//          (
//            (
//              samples[i]
//              )
//              )/4
//              ),SSD1306_WHITE); // zero volts is -128 because samples are chars not bytes
//          display.drawLine(i-128, SCREEN_HEIGHT/2, i-128, (SCREEN_HEIGHT/2)-(samples[i])/4,SSD1306_WHITE); // samples[i] is the real component, i+1 is imaginary
        display.drawLine(i-128, SCREEN_HEIGHT, i-128, (SCREEN_HEIGHT)-((sqrt(samples[i]*samples[i]+samples[i+1]*samples[i+1]))/2), SSD1306_WHITE); // get magnitude if the imaginary signal is going to matter
//      im[i-128] = 0;
      }
      sei();
    } else {
//      cli();
//      for(int i = 0;i<=127;i++){
//        Serial.print((byte)samples[i]+128);
//        Serial.print(' ');
//      }
//      Serial.println();
      
//      fix_fft(samples, im, m, 0); // first sample fft
////    sei();
//      for(byte j=0;j<126;j++){
//        samples[127-j]=-127;
//      }
////      for(byte j=0;j<100;j++){ // high pass filter
////        samples[j]=127;
////      }
//      fix_fft(samples, im, m, 1); // first sample inverse fft
//      for(int i = 0;i<=127;i++){
//        Serial.print((byte)samples[i]+128);
//        Serial.print(' ');
//      }
//      
//      Serial.println();
      for (byte i = 0; i <= 127; i++) {
//        display.drawLine(i, SCREEN_HEIGHT/2, i, (SCREEN_HEIGHT/2)-((samples[i])/5),SSD1306_WHITE); 
// drawing lines to negative numbers works fine, so this over+underflow is hapening in the output from fix_fft
// aka basically the difference between printing all the samples when first called, and printing them after inv(fft(sam))
// when the sample was close to the datatype limit already the variation can make it flip
//        display.drawLine(i, SCREEN_HEIGHT, i, SCREEN_HEIGHT-((sqrt(samples[i]*samples[i]+im[i]*im[i]))/3), SSD1306_WHITE);
//      im[i] = 0;
      }
    }
    
//    Serial.println();
//    if(secondSampleSet){
//      Serial.print("Second set ");
//    } else {
//      Serial.print("First set ");
//    }
//    Serial.println(bufferIndex);
//    for(int i=0;i<=255;i++){
//      Serial.print((byte)abs(samples[i]));
//      Serial.print(' ');
//    }
    display.display();
    bufferFull = false;
//    sei();
  }//bufferFull
}//loop


ISR(ADC_vect) {
  if (!bufferFull) { // fill the buffer until 128 samples including zero
    samples[bufferIndex] = ADCH - 128; // read 8 bits since left aligned ADC, results from 0 to 255. the subtraction fits byte in signed char (-128 to  127)

    bufferIndex++;
    if(bufferIndex==128){// if we are at 0 or 128, set the flag to start fft
      bufferFull = true;
      secondSampleSet=false;
    } else if(bufferIndex==0){
      bufferFull = true;
      secondSampleSet=true;
    }
  }
  TIFR1 |= _BV(ICF1);
}

void initTimerOne() {
  // credit:6v6gt
  //https://forum.arduino.cc/index.php?topic=584634.0
  // Initialise Timer1
  TCCR1A = 0;
  TCCR1B = _BV(CS10) |     // Bit 2:0 - CS12:0: Clock Select =  no prescaler
           _BV(WGM13) |    // WGM 12 = CTC ICR1 Immediate MAX
           _BV(WGM12);     // WGM 12 ditto

  ICR1 = 99; // TOP for 40kHz
}

void initADC() {
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
