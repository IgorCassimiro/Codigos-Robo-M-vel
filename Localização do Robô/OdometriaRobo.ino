#include <Ticker.h>
#include <Arduino.h>
#include <math.h>

#define ROTARY_DIREITA_PIN_A 13
#define ROTARY_DIREITA_PIN_B 14
#define ROTARY_ESQUERDA_PIN_A 27
#define ROTARY_ESQUERDA_PIN_B 25


// //################################## Variaveis Leitura Encoder Direita ############################

int64_t contPulsosRodaDireita=0;
int64_t contPulsosRodaDireitaAnterior=0;
uint8_t estadoRodaDireita=0;

// //################################## Variaveis Leitura Encoder Esquerda ############################

int64_t contPulsosRodaEsquerda=0;
int64_t contPulsosRodaEsquerdaAnterior=0;
uint8_t estadoRodaEsquerda=0;

// //################################## Variaveis da Odometria #######################################

const int dtLoopOdom_us = 1000000;                                                                     //base em micro segundos
double dtLoopOdom_s = dtLoopOdom_us / 1000000.0;

uint16_t pprRodaDireita = 16272; // PARAMETRO ROS
uint16_t pprRodaEsquerda = 16073; // PARAMETRO ROS

#define distancia_entre_eixos             0.41 // PARAMETRO ROS
#define raio_da_roda_direita              0.105 // PARAMETRO ROS
#define raio_da_roda_esquerda             0.105 // PARAMETRO ROS                                                            //está em cm
#define dist_entre_pulsos_roda_dir_em_cm  2 * M_PI * raio_da_roda_direita/(pprRodaDireita*1.0)
#define dist_entre_pulsos_roda_esq_em_cm  2 * M_PI * raio_da_roda_esquerda/(pprRodaEsquerda*1.0)

double x = 0.0;
double y = 0.0;
double theta = 0;
double xAnterior = 0.0;
double yAnterior = 0.0;
double thetaAnterior = 0.0;

double velRodaDireita = 0.000;
double velRodaEsquerda = 0.000;
double velLinear = 0.00;
double velAngular = 0.00;


portMUX_TYPE muxEncDireito = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxEncEsquerdo = portMUX_INITIALIZER_UNLOCKED;

Ticker blinker;

void IRAM_ATTR encoder_direito() {
  
  uint8_t s_d = estadoRodaDireita & 3;
  portENTER_CRITICAL_ISR(&muxEncDireito);
  
  if (digitalRead(ROTARY_DIREITA_PIN_A)) s_d |= 4;
  if (digitalRead(ROTARY_DIREITA_PIN_B)) s_d |= 8;
  switch (s_d) {
    case 0: case 5: case 10: case 15:
      break;
    case 1: case 7: case 8: case 14:
      contPulsosRodaDireita++; break;
    case 2: case 4: case 11: case 13:
      contPulsosRodaDireita--; break;
    case 3: case 12:
      contPulsosRodaDireita += 2; break;
    default:
      contPulsosRodaDireita -= 2; break;
  }
  estadoRodaDireita = (s_d >> 2);
  
  portEXIT_CRITICAL_ISR(&muxEncDireito);
}


void IRAM_ATTR encoder_esquerdo() { 
  
  uint8_t s_e = estadoRodaEsquerda & 3;

  portENTER_CRITICAL_ISR(&muxEncEsquerdo);
  
  if (digitalRead(ROTARY_ESQUERDA_PIN_A)) s_e |= 4;
  if (digitalRead(ROTARY_ESQUERDA_PIN_B)) s_e |= 8;
  switch (s_e) {
    case 0: case 5: case 10: case 15:
      break;
    case 1: case 7: case 8: case 14:
      contPulsosRodaEsquerda++; break;
    case 2: case 4: case 11: case 13:
      contPulsosRodaEsquerda--; break;
    case 3: case 12:
      contPulsosRodaEsquerda += 2; break;
    default:
      contPulsosRodaEsquerda -= 2; break;
  }
  estadoRodaEsquerda = (s_e >> 2);
  
  portEXIT_CRITICAL_ISR(&muxEncEsquerdo); 
}

void computaOdometria(){
  // Variáveis locais
  int difPulsosRodaDireita=0;
  int difPulsosRodaEsquerda=0;
  double distPercorridaRodaDireita = 0.00;
  double distPercorridaRodaEsquerda = 0.00;
  double deltaTheta = 0.00;
  double deltaDist = 0.00;

  vTaskEnterCritical(&muxEncDireito);
  difPulsosRodaDireita = contPulsosRodaDireita - contPulsosRodaDireitaAnterior;
  contPulsosRodaDireitaAnterior = contPulsosRodaDireita;
  vTaskExitCritical(&muxEncDireito);

  vTaskEnterCritical(&muxEncEsquerdo);
  difPulsosRodaEsquerda = contPulsosRodaEsquerda - contPulsosRodaEsquerdaAnterior;
  contPulsosRodaEsquerdaAnterior = contPulsosRodaEsquerda;
  vTaskExitCritical(&muxEncEsquerdo);

  distPercorridaRodaDireita = dist_entre_pulsos_roda_dir_em_cm * difPulsosRodaDireita;
  distPercorridaRodaEsquerda = dist_entre_pulsos_roda_esq_em_cm * difPulsosRodaEsquerda;

  deltaTheta = (distPercorridaRodaDireita - distPercorridaRodaEsquerda)/distancia_entre_eixos;
  deltaDist = (distPercorridaRodaDireita + distPercorridaRodaEsquerda)/2;

  x = xAnterior + deltaDist*(cos(theta+(deltaTheta/2)));
  y = yAnterior + deltaDist*(sin(theta+(deltaTheta/2)));
  theta = thetaAnterior + deltaTheta;

  velLinear = (deltaDist/dtLoopOdom_s);                                                                       //converte cm to m
  velAngular = deltaTheta/dtLoopOdom_s;

  velRodaEsquerda = ((difPulsosRodaEsquerda*dist_entre_pulsos_roda_esq_em_cm)/dtLoopOdom_s);          //converte em cm to m
  velRodaDireita = ((difPulsosRodaDireita*dist_entre_pulsos_roda_dir_em_cm)/dtLoopOdom_s);            //converte em cm to m

  xAnterior = x;
  yAnterior = y;
  thetaAnterior = theta;
}


void setup() {
  Serial.begin(115200);
  
  pinMode(ROTARY_DIREITA_PIN_A, INPUT_PULLUP);
  pinMode(ROTARY_DIREITA_PIN_B, INPUT_PULLUP);

  pinMode(ROTARY_ESQUERDA_PIN_A, INPUT_PULLUP);
  pinMode(ROTARY_ESQUERDA_PIN_B, INPUT_PULLUP);

  attachInterrupt(ROTARY_DIREITA_PIN_A, encoder_direito, CHANGE);
  attachInterrupt(ROTARY_DIREITA_PIN_B, encoder_direito, CHANGE);

  attachInterrupt(ROTARY_ESQUERDA_PIN_A, encoder_esquerdo, CHANGE);
  attachInterrupt(ROTARY_ESQUERDA_PIN_B, encoder_esquerdo, CHANGE);


  blinker.attach(1, computaOdometria);
  
}

void loop() {
 Serial.print("X: ");
 Serial.print(x);

 Serial.print("      ");

 Serial.print("Y: ");
 Serial.print(y);

 Serial.print("      ");

 Serial.print("THETA: ");
 Serial.print(theta);

 Serial.print("      ");

 Serial.print(" Vel linear: ");
 Serial.print(velLinear);

 Serial.print("      ");

 Serial.print(" Vel angular: ");
 Serial.print(velAngular);

 Serial.print("      ");

 Serial.print(" Vel esq: ");
 Serial.print(velRodaEsquerda);

 Serial.print("      ");

 Serial.print(" Vel dir: ");
 Serial.print(velRodaDireita);

 Serial.println("");

 
 delay(1000);

}

//
//#include <stdio.h>
//#include <string.h>
//#include <stdlib.h>
//
//
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "driver/gpio.h"
//#include "queueRoboROS.h"
//#include "driver/timer.h"
