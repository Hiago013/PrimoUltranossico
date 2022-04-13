//Incluindo biblioteca Ultrasonic.h
#include "Ultrasonic.h"

//Criando objeto ultrasonic e definindo as portas digitais
//do Trigger - 9 - e Echo - 10
Ultrasonic SensorUltrassonico1(9, 10);


long Microsegundos = 0;// Variável para armazenar o valor do tempo da reflexão do som refletido pelo objeto fornecido pela biblioteca do sensor
float DistanciaemCM = 0;// Variável para armazenar o valor da distância a ser convertido por uma função da própria bilbioteca do sensor
long DistanciaemCM_Filtrado = 0;// Variável para armazenar o valor da distância a ser convertido por uma função de filtragem (Media Móvel)

#define MotorLadoEsquerdo1 7
#define MotorLadoEsquerdo2 8

#define MotorLadoDireito1 A4
#define MotorLadoDireito2 A5


#define VelocidadeMotorLadoEsquerdo 11 // PWM
#define VelocidadeMotorLadoDireito 3  // PWM
#define STBY 2 // 

#define N 10 // Constante Auxiliar

#define distancia 20 // distancia para parar

#define vermelho 13
#define verde 12
#define azul A1

long values[N]; // Vetor para armazenar os valores do sensor


//============================================================ Escolhe a velocidade dos motores ==================================================================//
int ValorVelocidadeMotorLadoEsquerdo = 55; // Ajustar a velocidade do motor do lado esquerdo
int ValorVelocidadeMotorLadoDireito = 52; // Ajustar a velocidade do motor do lado direito

// ============================================= Prototipo de funções do motor =================================================================================================//


void BreakMotor(); // Para o motor

void TurnLeft();  // Gira para a esquerda

void TurnRight(); // Gira para a direita

void TurnBack(); // Gira para trás

void Forward(); // Anda para frente

void NewWay(); // Nova rota

void Yellow(); // Led Acende na cor amarelo

void Green(); // Led Acende na cor verde

void Red(); // Led acende na cor vermelho

long moving_average(long p_In); // Media movel


void setup() {

  //============================================================== Definições de entrada e saída ===================================================================//

  pinMode(MotorLadoEsquerdo1, OUTPUT);
  pinMode(MotorLadoEsquerdo2, OUTPUT);
  pinMode(MotorLadoDireito1, OUTPUT);
  pinMode(MotorLadoDireito2, OUTPUT);
  pinMode(vermelho, OUTPUT);
  pinMode(verde, OUTPUT);
  pinMode(azul, OUTPUT);

  Serial.begin(9600);// Inicia a comunicação seria com velocidade de 9600 bits por segundo

  delay(3000);// Tempo de espera para inicialização (para dar tempo de por o robô no chão)
}

void loop() {
  delay(20); // Tempo de espera para a leitura do sensor

  //Convertendo a distância em CM e lendo o sensor
  DistanciaemCM = SensorUltrassonico1.convert(SensorUltrassonico1.timing(), Ultrasonic::CM);
  DistanciaemCM_Filtrado = moving_average(DistanciaemCM);

  //Serial.print(DistanciaemCM);
  //Serial.println(" ");
  Serial.print(DistanciaemCM_Filtrado);
  Serial.println(" ");


  if (DistanciaemCM_Filtrado <= distancia) {// Se a distância lida pelo sensor for menor ou igual que 40 centimetros
    //Velocidade motor lado esquerdo
    analogWrite( VelocidadeMotorLadoEsquerdo, ValorVelocidadeMotorLadoEsquerdo);

    //Velocidade motor lado direito
    analogWrite( VelocidadeMotorLadoDireito, ValorVelocidadeMotorLadoDireito);

    Red();
    NewWay();

  }


  else {// Se não, ou seja, se a distância for maior que 40 centimetros

    //Velocidade motor lado esquerdo
    analogWrite( VelocidadeMotorLadoEsquerdo, ValorVelocidadeMotorLadoEsquerdo);

    //Velocidade motor lado direito
    analogWrite( VelocidadeMotorLadoDireito, ValorVelocidadeMotorLadoDireito);

    Forward();
    if(DistanciaemCM_Filtrado > 40) Green();
    else Blue();
  }

}

// ============================================= Funções  =================================================================================================//


void BreakMotor(){ // Essa função faz o motor parar de girar

    //
    digitalWrite(STBY, LOW);
    // Motor lado esquerdo desligado
    digitalWrite(MotorLadoEsquerdo1, LOW);
    digitalWrite(MotorLadoEsquerdo2, LOW);

    // Motor lado direito desligado
    digitalWrite(MotorLadoDireito1, LOW);
    digitalWrite(MotorLadoDireito2, LOW);
}


void TurnRight(){ // O cubeto gira à direita

    digitalWrite(STBY, HIGH);

    // Motor lado esquerdo para frente
    digitalWrite(MotorLadoEsquerdo1, LOW);
    digitalWrite(MotorLadoEsquerdo2, HIGH);


    // Motor lado direito para trás
    digitalWrite(MotorLadoDireito1, HIGH);
    digitalWrite(MotorLadoDireito2, LOW);

}

void TurnLeft(){ // O Cubeto gira à esquerda

  digitalWrite(STBY, HIGH);
  // Motor lado direito para frente
  digitalWrite(MotorLadoDireito1, LOW);
  digitalWrite(MotorLadoDireito2, HIGH);

  // Motor lado esquerdo trás
  digitalWrite(MotorLadoEsquerdo1, HIGH);
  digitalWrite(MotorLadoEsquerdo2, LOW); 

}

void TurnBack(){ // O Cubeto anda para trás

  digitalWrite(STBY, HIGH);

  // Motor lado direito para trás
    digitalWrite(MotorLadoDireito1, HIGH);
    digitalWrite(MotorLadoDireito2, LOW);

  // Motor lado esquerdo trás
    digitalWrite(MotorLadoEsquerdo1, HIGH);
    digitalWrite(MotorLadoEsquerdo2, LOW); 

}

void Forward(){ // O Cubeto anda para frente

  digitalWrite(STBY, HIGH);

  // Motor lado direito para frente
    digitalWrite(MotorLadoDireito1, LOW);
    digitalWrite(MotorLadoDireito2, HIGH);

  // Motor lado esquerdo para frente
    digitalWrite(MotorLadoEsquerdo1, LOW);
    digitalWrite(MotorLadoEsquerdo2, HIGH);

}

void NewWay(){ // Determinar uma nova rota
  DistanciaemCM = SensorUltrassonico1.convert(SensorUltrassonico1.timing(), Ultrasonic::CM);
  DistanciaemCM_Filtrado = moving_average(DistanciaemCM);
  while (DistanciaemCM_Filtrado < distancia)
  {
    BreakMotor();
    delay(500);
    //TurnBack();
    //delay(700);
    //BreakMotor();
    //delay(500);  
    TurnRight();
    delay(400);
    BreakMotor();
    delay(500);
    DistanciaemCM = SensorUltrassonico1.convert(SensorUltrassonico1.timing(), Ultrasonic::CM);
    DistanciaemCM_Filtrado = moving_average(DistanciaemCM);
    if (DistanciaemCM_Filtrado < distancia){
      TurnLeft();
      delay(800);
      BreakMotor();
      delay(500);
    } 
    else break;
    DistanciaemCM = SensorUltrassonico1.convert(SensorUltrassonico1.timing(), Ultrasonic::CM);
    DistanciaemCM_Filtrado = moving_average(DistanciaemCM);

    if (DistanciaemCM_Filtrado < distancia) {
      TurnLeft();
      delay(300);
      BreakMotor();
      delay(500);
      }
    else break;
    DistanciaemCM = SensorUltrassonico1.convert(SensorUltrassonico1.timing(), Ultrasonic::CM);
    DistanciaemCM_Filtrado = moving_average(DistanciaemCM);
  }

}

long moving_average(long p_In) // Media movel de 10 periodos
{
  int i;
  long adder = 0; // acumulador para somar os pontos da media movel
  // desloca os elementos do vetor media movel 
  for(i = N; i>0; i--) values[i] = values[i-1];
  
  values[0] = p_In; // posicao inicial do vetor recebe a leitra p_In

  for(i = 0; i<N; i++) adder += values[i]; // soma os elementos do vetor
  
  return adder/N; // retorna a media
}


void Blue()
{
digitalWrite(vermelho, LOW);
digitalWrite(verde, LOW);
digitalWrite(azul, HIGH);
}

void Green()
{
digitalWrite(vermelho, LOW);
digitalWrite(verde, HIGH);
digitalWrite(azul, LOW);
}

void Red()
{
digitalWrite(vermelho, HIGH);
digitalWrite(verde, LOW);
digitalWrite(azul, LOW);
}