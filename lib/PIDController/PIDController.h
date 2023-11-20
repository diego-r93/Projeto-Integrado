#ifndef PIDCONTROLLER
#define PIDCONTROLLER

class Pid {
  public:
   Pid();
   float pid_control(float input, float target);

   void setKp(double value);
   void setKi(double value);
   void setKd(double value);

   double getKp();
   double getKi();
   double getKd();

   float get_lastError();
   float get_lastSpeed();
   float get_pid();

  private:
   double kp;
   double ki;
   double kd;
   
   unsigned long last_run;
   float last_error;
   float last_speed;
   float proportional;
   float integral;
   float derivative;
};

#endif