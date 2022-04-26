#include <Arduino.h>
#include "UltraSonic.h"
#include "MPU6050_light.h"

struct PonteH
{
int pin_motor_left_1; 
int pin_motor_left_2; 

int pin_motor_right_1; 
int pin_motor_right_2; 

int STBY;

int pin_speed_motor_left;
int pin_speed_motor_right;
};

class PID{
  private:
  float kp, ki, kd;
  float error, integral, derivative, last_error, setPoint;
  float output;
  public:
  PID(float _kp, float _ki, float _kd);
  float calculate(float input,float _setPoint);
};

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

class ControlMotor{
  private:
  int pin_motor_left_1, pin_motor_right_1, pin_motor_left_2, pin_motor_right_2;
  int pin_motor_left_pwm, pin_motor_right_pwm;
  int pin_STBY;

  public:
  ControlMotor(PonteH _pins); //constructor
  void setPWM(int _pwm_left, int _pwm_right);
  void goForward();
  void TurnLeft();
  void TurnRight();
  void Off();

};

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

class LedRGB{
  private:
  int pin_r, pin_g, pin_b;
  public:
  LedRGB(int _pin_r, int _pin_g, int _pin_b);
  void setR();
  void setG();
  void setB();
  void Off();
};

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

//----------------------------------------------------Definindos Conexoes Ponte H<->Motor----------------------------------------------------
#define pin_motor_left_1 4 // AIN 1
#define pin_motor_left_2 5 // AIN 2

#define pin_motor_right_1 7 // BIN 1
#define pin_motor_right_2 8 // BIN 2


#define pin_speed_motor_left 6 // PWMA
#define pin_speed_motor_right 9  // PWMB
#define STBY 2 //

#define defaultPWM 60
// ----------------------------------------------------Definindos Conexoes do HCSR04----------------------------------------------------
int distancia; //VARIÁVEL DO TIPO INTEIRO
String result; //VARIÁVEL DO TIPO STRING

int trigPin = 3;
int echoPin = 10;
Ultrasonic ultrasonic(trigPin,echoPin);
//----------------------------------------------------Instanciando Objetos----------------------------------------------------
PonteH set_linkage = {pin_motor_left_1, pin_motor_left_2, pin_motor_right_1, pin_motor_right_2,
                  STBY, pin_speed_motor_left, pin_speed_motor_right}; // Ponte H pinagens

ControlMotor linkage(set_linkage); // Ponte H motor

Ultrasonic ultrasonicsensor(3, 10); // Ultrassonico

PID pid(3, 2, 2);

MPU6050 mpu(Wire);

LedRGB leds(13, 11, 12);


// ----------------------------------------------------Variaveis Globais----------------------------------------------------
int distance; // Variavel para armazenar a distancia
unsigned long timer = 0; // Variavel para armazenar o tempo
float angles[3]; // Variavel para armazenar os angulos
float auxForward, auxRight, auxLeft; // Variavel para armazenar o angulo
// ----------------------------------------------------Protótipo de funções----------------------------------------------------

int hcsr04_distance(); // Função para leitura do HCSR04
void new_way(); // Função para o novo caminho
void get_angle(); // Função para leitura do yaw
float pid2pwm(float pid, int max); // Função para transformar a saída do controlador de graus para pwm

// ----------------------------------------------------Função principal----------------------------------------------------

void setup() {
  
  Serial.begin(9600);
  Wire.begin();
  byte status = mpu.begin(1, 0);
  while(status != 0){ }
  mpu.calcOffsets(true, true);
  mpu.setFilterGyroCoef(0.98);
  linkage.Off();
  linkage.setPWM(defaultPWM, defaultPWM);
  delay(3000); // Delay para por o robo em posição
  get_angle();
  auxForward = angles[2];
}

void loop() {
  get_angle();
  if((abs(angles[0]) > 30) || (abs(angles[1]) > 30)){
    linkage.Off();
    linkage.setPWM(0, 0);
    leds.Off();
    while ((abs(angles[0]) > 30) || (abs(angles[1]) > 30)) {get_angle(); Serial.println("Aguardando estar no chão."); delay(20);}
    get_angle();
    auxForward = angles[2];
  }
  else{
    float out = pid.calculate(angles[2], auxForward); // Calcula o PID
      if(out < 0) {out = pid2pwm(out, 10); linkage.setPWM(defaultPWM + out, defaultPWM - out);} // Se o PID for negativo, o motor gira mais para a esquerda
      else        {out = pid2pwm(out, 10); linkage.setPWM(defaultPWM - out, defaultPWM + out);} // Se o PID for positivo, o motor gira mais para a direita
    
    distance = hcsr04_distance();
    if(distance <= 20){
      leds.setR();
      linkage.Off();
      delay(1000);
      new_way();
      get_angle();
      auxForward = angles[2];
      auxRight = angles[2];
    }

    else{
      linkage.goForward();
      if (distance > 40) leds.setG();
      else leds.setB();
    }

    delay(65); // Delay para não sobrecarregar o HCSR04
  
  }
}


// ----------------------------------------------------Implementação das funções----------------------------------------------------
int hcsr04_distance(){ // Função para leitura do HCSR04
  return ultrasonic.convert(ultrasonic.timing(), Ultrasonic::CM);
}
void new_way(){ // Função para o novo caminho
  int aux = 1;
  do
  {
      switch (aux)
      {
      case 1:
        while (auxRight - 90 - angles[2] < 0){
          linkage.TurnRight();
          get_angle();
          delay(10);
        }
        linkage.Off();
        delay(500);
        get_angle();
        auxLeft = angles[2];
        distance = hcsr04_distance();
        break;

      case 2:
        while (auxLeft + 180 - angles[2] > 0){
          linkage.TurnLeft();
          get_angle();
          delay(10);
        }
        linkage.Off();
        delay(500);
        get_angle();
        auxLeft = angles[2];
        distance = hcsr04_distance();
        break;

      case 3:
        while (auxLeft + 90 - angles[2] > 0){
            linkage.TurnLeft();
            get_angle();
            delay(10);
          }
        linkage.Off();
        delay(500);
        get_angle();
        auxRight = angles[2];
        distance = hcsr04_distance();
        break;
      
      }
      aux += 1;
      if(aux>3) aux = 0;
  } while (distance <= 20);
  
}

void get_angle(){ // Função para leitura do roll, pitch e yaw
  mpu.update();
  if((millis() - timer) > 10){
    angles[0] = mpu.getAngleX();
    angles[1] = mpu.getAngleY();
    angles[2] = mpu.getAngleZ();
    timer = millis();
  }
}

float pid2pwm(float pid, int max){ // Função para transformar a saída do controlador de graus para pwm
  if(pid < 0){
    pid = map(pid, 0, -360, 0, 200);
    if(pid >= max) pid = max;
  }
  else
  {
    pid = map(pid, 0, 360, 0, 200);
    if(pid >= max) pid = max;
  }

  return pid;
}