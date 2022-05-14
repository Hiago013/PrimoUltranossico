#include "Arduino.h"
#include "LEDRGB.h"

LedRGB::LedRGB(int _pin_r, int _pin_g, int _pin_b){
  pin_r = _pin_r;
  pin_g = _pin_g;
  pin_b = _pin_b;
  pinMode(pin_r, OUTPUT);
  pinMode(pin_g, OUTPUT);
  pinMode(pin_b, OUTPUT);
}

void LedRGB::setR(){
  digitalWrite(pin_r, HIGH);
  digitalWrite(pin_g, LOW);
  digitalWrite(pin_b, LOW);
}

void LedRGB::setG(){
  digitalWrite(pin_r, LOW);
  digitalWrite(pin_g, HIGH);
  digitalWrite(pin_b, LOW);
}

void LedRGB::setB(){
  digitalWrite(pin_r, LOW);
  digitalWrite(pin_g, LOW);
  digitalWrite(pin_b, HIGH);
}

void LedRGB::Off(){
  digitalWrite(pin_r, LOW);
  digitalWrite(pin_g, LOW);
  digitalWrite(pin_b, LOW);
}