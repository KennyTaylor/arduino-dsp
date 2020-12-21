#include "fix_fft.h"


const int n=256;
const int m=8;
volatile char samples[n],im[n];
volatile byte bufferIndex=0;
volatile bool bufferFull=true;


void setup() {  
  pinMode(5,OUTPUT); // testing
  analogWrite(5,128);
  Serial.begin(2000000);
  cli();
  initTimerOne();
  initADC();
  sei() ;
}

void loop() {
  if(bufferFull){
    cli();
//    for(int i=0;i<n;i++){
//      Serial.print((int)samples[i]);
//      Serial.print(' ');
//    }
    fix_fft(samples,im,m,0);
    Serial.print('\n');
    for(int i=0;i<n;i++){
      Serial.print(sqrt(samples[i]*samples[i]+im[i]*im[i]));
      Serial.print('\t');
      im[i]=0;
    }
    bufferFull=false;
    sei();
  }
}


ISR(ADC_vect) {
  if(!bufferFull){ // fill the buffer until 256 samples including zero
    samples[bufferIndex]=ADCH-128; // read 8 bits since left aligned ADC, results from 0 to 255. the subtraction fits byte in signed char (-128 to  127)
    
    if(bufferIndex++==(n-1)){ // increment buffer, stop sampling when i==255
      bufferFull=true;
    }
  }
  TIFR1|= _BV(ICF1);
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
