#ifndef THERMISTOR_H
#define THERMISTOR_H

#include <Arduino.h>

#include "MovingAverageFilter.h"

class Thermistor {
  public:
   Thermistor(uint8_t pin, uint16_t nomRes, uint16_t bCoef, uint16_t serialRes, int sampleCount, float tempNominal);
   void update();
   float getTemperature() const;

  private:
   uint8_t _pin;
   uint16_t _nominalResistance;
   uint16_t _bCoefficient;
   uint16_t _serialResistance;
   float _temperatureNominal;
   float _temperature;
   MovingAverageFilter _filter;
};

#endif
