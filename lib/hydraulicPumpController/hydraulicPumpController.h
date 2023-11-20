#ifndef _PUMP_
#define _PUMP_

#include <Arduino.h>
#include <ArduinoJson.h>

#include <set>

#include "freeRTOSTimerController.h"

#define MAX_SIZE_DOCUMENT 4096

class HydraulicPumpController {
  public:
   const char *pumperCode;
   const uint8_t gpioPin;

   TickType_t *pulseDurationPointer = &pulseDuration;

   HydraulicPumpController(const char *pumperCode, uint8_t gpioPin, TickType_t pulseDuration);

   bool getPumpState();

   void startPump();
   void stopPump();

   std::set<String> getDriveTimes();
   std::set<String> *getDriveTimesPointer();

   DynamicJsonDocument getJsonData();
   DynamicJsonDocument *getJsonDataPointer();

   TickType_t getPulseDuration();
   void setPulseDuration(TickType_t);

  private:
   std::set<String> driveTimes;
   DynamicJsonDocument jsonData;
   FreeRTOSTimer timer;

   TickType_t pulseDuration;

   bool pumpState = false;

   static void pumpControlCallback(TimerHandle_t xTimer);
};

#endif