#include "TDSMeter.h"

TDSMeter::TDSMeter(uint8_t pin, float vref, int scount, float temperature, float cellConstant)
    : _pin(pin), _vref(vref), _scount(scount), _temperature(temperature), _tdsValue(0), _filter(scount), _cellConstant(cellConstant) {
   pinMode(_pin, INPUT);
}

void TDSMeter::update() {
   int sensorValue = analogRead(_pin);  // Lê o valor do sensor
   _filter.addValue(sensorValue);       // Adiciona o valor à filtragem

   const float _calibrationOffset = 0.0;  // Constante para armazenar o offset de calibração

   float averageSensorValue = _filter.getAverage();  // Calcula a média das leituras

   // Calcula a tensão analógica baseada na leitura média do ADC
   float voltage = (averageSensorValue / 4096.0) * _vref;  // _vref é a tensão de referência do ADC

   // Adiciona o offset de calibração à tensão
   // O valor de _calibrationOffset deve ser ajustado durante o processo de calibração
   float calibratedVoltage = voltage - _calibrationOffset;

   // Compensação de temperatura
   float compensationCoefficient = 1.0 + 0.02 * (_temperature - 25.0);
   float compensationVoltage = calibratedVoltage / compensationCoefficient;  

   // Calcula a condutividade em microsiemens por centímetro (uS/cm)
   // A equação é: Condutividade = (Tensão Compensada / Constante da Célula) * 1000
   // Onde a Constante da Célula é um valor específico do seu sensor que deve ser conhecido
   _tdsValue = (compensationVoltage / _cellConstant) * 1000;  // Multiplica por 1000 para converter de mS/cm para uS/cm
}

float TDSMeter::getTDSValue() const {
   return _tdsValue;
}