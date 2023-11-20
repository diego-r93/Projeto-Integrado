#include <PIDController.h>

#include "Arduino.h"
#include "freertos/FreeRTOS.h"

#define PID_SAMPLE_TIME 100

Pid::Pid() {
   last_run = 0;
   last_error = 0;
   proportional = 0.0;
   integral = 0.0;
   derivative = 0.0;
   kp = 0.0;   // Valor ajustado
   ki = 0.0;   // Valor ajustado
   kd = 15.0;  // Valor ajustado
}

float Pid::pid_control(float input, float target) {
   TickType_t now = xTaskGetTickCount();
   TickType_t dt = (now - last_run) * portTICK_PERIOD_MS / 1000;

   if (last_run == 0 || dt == 0) {
      dt = PID_SAMPLE_TIME;  // Assume um intervalo padrão na primeira execução
   }

   double error = target - input;
   proportional = kp * error;

   integral += (ki * error * dt);

   // Implementação do Anti-Windup: limitando o termo integral
   integral = fmin(fmax(integral, 0), target);

   derivative = kd * ((error - last_error) / dt);

   last_error = error;
   last_run = now;

   return (proportional + integral + derivative);
}

void Pid::setKp(double value) { kp = value; };

void Pid::setKi(double value) { ki = value; };

void Pid::setKd(double value) { kd = value; };

double Pid::getKp() { return kp; };

double Pid::getKi() { return ki; };

double Pid::getKd() { return kd; };

float Pid::get_lastError() { return last_error; };

float Pid::get_lastSpeed() { return last_speed; };

float Pid::get_pid() { return (proportional + integral + derivative); };
