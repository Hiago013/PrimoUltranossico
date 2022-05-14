#ifndef PID_H
#define PID_H

#include "Arduino.h"


class PID{
  private:
  float kp, ki, kd;
  float error, integral, derivative, last_error, setPoint;
  float output;
  public:
  PID(float _kp, float _ki, float _kd);
  float calculate(float input,float _setPoint);
};


#endif

