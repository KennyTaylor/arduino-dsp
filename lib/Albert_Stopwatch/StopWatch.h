/* 
Simple stopwatch library for the Arduino.
Albert van Dalen http://www.avdweb.nl
*/

#ifndef Stopwatch_h
#define Stopwatch_h

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

class Stopwatch 
{
public:
    Stopwatch(unsigned long (*timeFunctionPtr)()); 
    void start();
    void stop(); 
    unsigned long maxInterval(bool reset=0); 

    unsigned long counter, interval; 

private:
    unsigned long (*timeFunctionPtr)();
    
    unsigned long starttime, _maxInterval;    
};
#endif

