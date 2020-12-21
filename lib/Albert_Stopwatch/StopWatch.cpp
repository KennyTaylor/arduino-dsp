/* 
Simple stopwatch library for the Arduino.
Albert van Dalen http://www.avdweb.nl
Version 20-4-2013
*/

#include "StopWatch.h"

Stopwatch::Stopwatch(unsigned long (*timeFunctionPtr)()): 
timeFunctionPtr(timeFunctionPtr)
{ 
}

void Stopwatch::start()
{ starttime = timeFunctionPtr();
  counter++; 
}

void Stopwatch::stop()
{ interval = timeFunctionPtr() - starttime; 
  _maxInterval = max(_maxInterval, interval); 
}

unsigned long Stopwatch::maxInterval(bool reset) 
{ unsigned long returnValue = _maxInterval;
  if(reset) _maxInterval = 0;
  return returnValue;
}


