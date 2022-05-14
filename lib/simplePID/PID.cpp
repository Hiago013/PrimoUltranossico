#include "Arduino.h"
#include "PID.h"

PID::PID(float _kp, float _ki, float _kd){
  kp = _kp;
  ki = _ki;
  kd = _kd;
  error = 0;
  integral = 0;
  derivative = 0;
  last_error = 0;
  output = 0;
}

float PID::calculate(float input, float _setPoint){
  setPoint = _setPoint;
  error = setPoint - input;
  integral += error;
  derivative = error - last_error;
  last_error = error;
  output = kp * error + ki * integral + kd * derivative;
  return output;
}