#ifndef TDSMETER_H
#define TDSMETER_H

#include <Arduino.h>

#include "MovingAverageFilter.h"

class TDSMeter {
  public:
   TDSMeter(uint8_t pin, float vref, int scount, float temperature, float cellConstant);
   void update();
   float getTDSValue() const;

  private:
   uint8_t _pin;
   float _vref;
   int _scount;
   float _temperature;
   float _tdsValue;
   float _cellConstant;
   MovingAverageFilter _filter;
};

#endif
