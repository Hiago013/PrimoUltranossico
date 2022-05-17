#include <Arduino.h>
#include "UltraSonic.h"
#include "MPU6050_light.h"
#include "PID.h"
#include "ControlMotor.h"
#include "LEDRGB.h"

//----------------------------------------------------Definindos Conexoes Ponte H<->Motor----------------------------------------------------
#define pin_motor_left_1 4 // AIN 1
#define pin_motor_left_2 5 // AIN 2

#define pin_motor_right_1 7 // BIN 1
#define pin_motor_right_2 8 // BIN 2

#define pin_speed_motor_left 6 // PWMA
#define pin_speed_motor_right 9  // PWMB
#define STBY 2 // STBY

#define defaultPWM 60 // PWM default
// ----------------------------------------------------Definindos Conexoes do HCSR04----------------------------------------------------

int trigPin = 3; // Trigger
int echoPin = 10; // Echo
Ultrasonic ultrasonic(trigPin,echoPin); // Instanciando objeto ultrasonic

//----------------------------------------------------Definindo Conexoes com a Ponte H----------------------------------------------------


PonteH set_linkage = {pin_motor_left_1, pin_motor_left_2, pin_motor_right_1, pin_motor_right_2,
                  STBY, pin_speed_motor_left, pin_speed_motor_right}; // Ponte H pinagens

ControlMotor linkage(set_linkage); // Ponte H motor

//---------------------------------------------------Instanciando objetos ----------------------------------------------------
PID pid(3, 2, 2); // PID(Kp, Ki, Kd)

MPU6050 mpu(Wire); // MPU6050 sensor

LedRGB leds(13, 11, 12); // LED RGB -> red - 13, green - 11, blue - 12


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
  get_angle(); // Leitura do angulo yaw, pitch e roll
  if((abs(angles[0]) > 30) || (abs(angles[1]) > 30)){ // Se o angulo pitch ou roll for maior que 30 graus, o robo fica parado
    linkage.Off();                                    // pois é considerado que ele caiu
    linkage.setPWM(0, 0);
    leds.Off();
    while ((abs(angles[0]) > 30) || (abs(angles[1]) > 30)) {get_angle(); Serial.println("Aguardando estar no chão."); delay(20);}
    get_angle();                                      // Leitura do angulo yaw, pitch e roll
    auxForward = angles[2];                           // Armazenando o angulo forward
  }
  
  else{
    float out = pid.calculate(angles[2], auxForward); // Calcula o PID
      if(out < 0) {out = pid2pwm(out, 10); linkage.setPWM(defaultPWM + out, defaultPWM - out);} // Se o PID for negativo, o motor gira mais para a esquerda
      else        {out = pid2pwm(out, 10); linkage.setPWM(defaultPWM - out, defaultPWM + out);} // Se o PID for positivo, o motor gira mais para a direita
    
    distance = hcsr04_distance(); // Leitura da distancia
    if(distance <= 20){ // Se a distancia for menor que 20 cm, o robo fica parado e acende o LED vermelho
      leds.setR();
      linkage.Off();
      delay(1000);
      new_way();
      get_angle();
      auxForward = angles[2];
      auxRight = angles[2];
    }

    else{ // Se a distancia for maior que 20 cm, o robo continua a correr
      linkage.goForward();
      if (distance > 40) leds.setG(); // Se a distancia for maior que 40 cm, o LED verde é acendido
      else leds.setB(); // Se a distancia for menor que 40 cm, o LED azul é acendido
    }

    unsigned long currentTime = millis(); // Armazena o tempo atual
    while((millis() - currentTime) < 50){ }// Espera 50 milisegundos
  
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
      case 1: // Primeiro caso tenta girar 90 graus sentido horario
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

      case 2: // Segundo caso tentar girar 180 graus sentido anti-horario
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

      case 3: // Terceiro caso tentar girar 90 graus sentido anti-horario
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