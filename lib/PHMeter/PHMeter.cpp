#include "PHMeter.h"

PHMeter::PHMeter(uint8_t pin, float vref, int scount) : _pin(pin), _vref(vref), _scount(scount), _phValue(0), _filter(scount) {
   pinMode(_pin, INPUT);
}

void PHMeter::update() {
   int sensorValue = analogRead(_pin);  // Lê o valor do sensor
   _filter.addValue(sensorValue);       // Adiciona o valor à filtragem

   const float _calibrationOffset = 1.7;  // Constante para armazenar o offset de calibração

   float averageSensorValue = _filter.getAverage();  // Calcula a média das leituras

   // Calcula a tensão analógica baseada na leitura média do ADC
   float voltage = (averageSensorValue / 4096.0) * _vref;  // _vref é a tensão de referência do ADC

   // Adiciona o offset de calibração à tensão
   // O valor de _calibrationOffset deve ser ajustado durante o processo de calibração
   float calibratedVoltage = voltage - _calibrationOffset;

   // Calcula o pH usando a equação de Nernst:
   // pH = pH_zero - (Voltagem Calibrada do Eletrodo / Slope)
   // Onde pH_zero é o ponto de calibração (geralmente em torno de 7) e
   // Slope é a inclinação da resposta do eletrodo (idealmente 59.16 mV/pH a 25°C)
   _phValue = 7 - (calibratedVoltage / 0.05916);  // A inclinação (slope) é aproximadamente 59.16 mV por unidade de pH

   // Restringe o valor do pH a um intervalo de 0 a 14
   _phValue = constrain(_phValue, 0, 14);
}

float PHMeter::getPHValue() const {
   return _phValue;
}
