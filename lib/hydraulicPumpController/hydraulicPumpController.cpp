#include "hydraulicPumpController.h"

HydraulicPumpController::HydraulicPumpController(const char *pumperCode, uint8_t gpioPin, TickType_t pulseDuration)
    : gpioPin(gpioPin),
      jsonData(MAX_SIZE_DOCUMENT),
      timer("PumpTimer", pulseDuration, pdFALSE, (void *)this, &HydraulicPumpController::pumpControlCallback) {
   this->pumperCode = pumperCode;
   this->pulseDuration = pulseDuration;
   pinMode(gpioPin, OUTPUT);

   // Caso o Esp32 reinicie enquanto um timer estiver ativo
   if (digitalRead(gpioPin) == HIGH)
      digitalWrite(gpioPin, LOW);
}

bool HydraulicPumpController::getPumpState() {
   return pumpState;
}

void HydraulicPumpController::startPump() {
   pumpState = true;
   digitalWrite(gpioPin, HIGH);
   timer.start();
}

void HydraulicPumpController::stopPump() {
   timer.stop();
   digitalWrite(gpioPin, LOW);
   pumpState = false;
}

std::set<String> HydraulicPumpController::getDriveTimes() {
   return driveTimes;
}

std::set<String> *HydraulicPumpController::getDriveTimesPointer() {
   return &driveTimes;
}

DynamicJsonDocument HydraulicPumpController::getJsonData() {
   return jsonData;
}

DynamicJsonDocument *HydraulicPumpController::getJsonDataPointer() {
   return &jsonData;
}

void HydraulicPumpController::pumpControlCallback(TimerHandle_t xTimer) {
   HydraulicPumpController *controller = (HydraulicPumpController *)pvTimerGetTimerID(xTimer);

   controller->stopPump();
}

TickType_t HydraulicPumpController::getPulseDuration() {
   return pulseDuration;
}

void HydraulicPumpController::setPulseDuration(TickType_t newPulseDuration) {
   pulseDuration = newPulseDuration;
}
