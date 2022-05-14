#include "Arduino.h"
#include "ControlMotor.h"

ControlMotor::ControlMotor(PonteH _pins){
  pin_motor_left_1 = _pins.pin_motor_left_1;
  pin_motor_right_1 = _pins.pin_motor_right_1;
  pin_motor_left_2 = _pins.pin_motor_left_2;
  pin_motor_right_2 = _pins.pin_motor_right_2;
  pin_motor_left_pwm = _pins.pin_speed_motor_left;
  pin_motor_right_pwm = _pins.pin_speed_motor_right;
  pin_STBY = _pins.STBY;
  pinMode(pin_motor_left_1, OUTPUT);
  pinMode(pin_motor_left_2, OUTPUT);
  pinMode(pin_motor_right_1, OUTPUT);
  pinMode(pin_motor_right_2, OUTPUT);
  pinMode(_pins.STBY, OUTPUT);
  pinMode(pin_motor_left_pwm, OUTPUT);
  pinMode(pin_motor_right_pwm, OUTPUT);
}

void ControlMotor::setPWM(int _pwm_left, int _pwm_right){
  analogWrite(pin_motor_left_pwm, _pwm_left);
  analogWrite(pin_motor_right_pwm, _pwm_right);
}

void ControlMotor::goForward(){
  digitalWrite(pin_STBY, HIGH);
  digitalWrite(pin_motor_right_1, LOW);
  digitalWrite(pin_motor_right_2, HIGH);
  digitalWrite(pin_motor_left_1, LOW);
  digitalWrite(pin_motor_left_2, HIGH);
}

void ControlMotor::TurnLeft(){
  digitalWrite(pin_STBY, HIGH);
  digitalWrite(pin_motor_right_1, LOW);
  digitalWrite(pin_motor_right_2, HIGH);
  digitalWrite(pin_motor_left_1, HIGH);
  digitalWrite(pin_motor_left_2, LOW);
}

void ControlMotor::TurnRight(){
  digitalWrite(pin_STBY, HIGH);
  digitalWrite(pin_motor_right_1, HIGH);
  digitalWrite(pin_motor_right_2, LOW);
  digitalWrite(pin_motor_left_1, LOW);
  digitalWrite(pin_motor_left_2, HIGH);
}

void ControlMotor::Off(){
  digitalWrite(pin_STBY, LOW);
  digitalWrite(pin_motor_right_1, LOW);
  digitalWrite(pin_motor_right_2, LOW);
  digitalWrite(pin_motor_left_1, LOW);
  digitalWrite(pin_motor_left_2, LOW);
}