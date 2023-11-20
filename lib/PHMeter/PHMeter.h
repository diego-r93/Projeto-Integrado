#ifndef PHMETER_H
#define PHMETER_H

#include <Arduino.h>

#include "MovingAverageFilter.h"

class PHMeter {
  public:
   PHMeter(uint8_t pin, float vref, int scount);
   void update();
   float getPHValue() const;

  private:
   uint8_t _pin;
   float _vref;
   int _scount;
   float _phValue;
   MovingAverageFilter _filter;
};

#endif