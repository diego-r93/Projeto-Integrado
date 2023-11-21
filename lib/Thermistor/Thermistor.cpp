#include "Thermistor.h"

Thermistor::Thermistor(uint8_t pin, uint16_t nomRes, uint16_t bCoef, uint16_t serialRes, int sampleCount, float tempNominal)
    : _pin(pin), _nominalResistance(nomRes), _bCoefficient(bCoef), _serialResistance(serialRes), _temperatureNominal(tempNominal), _temperature(0), _filter(sampleCount) {
   pinMode(_pin, INPUT);
}
void Thermistor::update() {
   _filter.addValue(analogRead(_pin));
   float average = _filter.getAverage();

   // Converte o valor para resistência
   average = 4096.0 / average - 1;
   average = _serialResistance / average;

   // Calcula a temperatura usando a Equação de Steinhart-Hart:
   // 1/T = 1/To + 1/B * ln(R/Ro)
   // onde To é a temperatura nominal (em Kelvin), B é o coeficiente B do termistor,
   // Ro é a resistência nominal do termistor (em Ohms) à temperatura nominal To,
   // e R é a resistência atual do termistor. T é a temperatura em Kelvin.
   float steinhart;
   steinhart = average / _nominalResistance;           // (R/Ro)
   steinhart = log(steinhart);                         // ln(R/Ro)
   steinhart /= _bCoefficient;                         // 1/B * ln(R/Ro)
   steinhart += 1.0 / (_temperatureNominal + 273.15);  // + (1/To)
   steinhart = 1.0 / steinhart;                        // Inverter para obter T em Kelvin
   _temperature = steinhart - 273.15;                  // Converter T de Kelvin para Celsius
}

float Thermistor::getTemperature() const {
   return _temperature;
}
