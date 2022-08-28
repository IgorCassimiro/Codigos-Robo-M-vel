  #define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

const int InA1 = 23;
const int InB1 = 22;
const int PWM1 = 21;
const int InA2 = 19;
const int InB2 = 18;
const int PWM2 = 5;

const int PWM_Chan1 = 0;
const int PWM_Chan2 = 1;
const int PWM_Freq1 =10000;
const int PWM_Freq2 =10000;
const int PWM_Res1 = 16;
const int PWM_Res2 = 16;

void setup(){

  Serial.begin(115200);
  Dabble.begin("MyESP32");

  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);

  ledcSetup(PWM_Chan1, PWM_Freq1, PWM_Res1);
  ledcSetup(PWM_Chan1, PWM_Freq2, PWM_Res2);

  ledcAttachPin(PWM1, PWM_Chan1);
  ledcAttachPin(PWM2, PWM_Chan2);
}

void loop(){
  Dabble.processInput();
  Serial.print("Key Pressed: ");
  if(GamePad.isUpPressed())
  {
    Serial.print("Up");
    moveFrente(32765);
  } else if(GamePad.isDownPressed()){
    Serial.print("Down");
    moveTras(32765);
  } else if(GamePad.isRightPressed()){
    Serial.print("Right");
    moveDireita(32765);
  }else if(GamePad.isLeftPressed()){
    Serial.print("Left");
    moveEsquerda(32765);
  } else{
    parar();
  }
}

void moveTras(int vel){
  digitalWrite(InA1, HIGH);
  digitalWrite(InB1, LOW);
  ledcWrite(PWM_Chan1, vel);

  digitalWrite(InA2, HIGH);
  digitalWrite(InB2, LOW);
  ledcWrite(PWM_Chan2, vel);
}

void moveEsquerda(int vel){
  digitalWrite(InA1, HIGH);
  digitalWrite(InB1, LOW);
  ledcWrite(PWM_Chan1, vel);

  digitalWrite(InA2, LOW);
  digitalWrite(InB2, HIGH);
  ledcWrite(PWM_Chan2, vel);
}

void moveDireita(int vel){
  digitalWrite(InA1, LOW);
  digitalWrite(InB1, HIGH);
  ledcWrite(PWM_Chan1, vel);

  digitalWrite(InA2, HIGH);
  digitalWrite(InB2, LOW);
  ledcWrite(PWM_Chan2, vel);
}

void moveFrente(int vel){
  digitalWrite(InA1, LOW);
  digitalWrite(InB1, HIGH);
  ledcWrite(PWM_Chan1, vel);

  digitalWrite(InA2, LOW);
  digitalWrite(InB2, HIGH);
  ledcWrite(PWM_Chan2, vel);
}



void parar(){
  digitalWrite(InA1, LOW);
  digitalWrite(InB1, LOW);
  ledcWrite(PWM_Chan1, 0);
  
  digitalWrite(InA2, LOW);
  digitalWrite(InB2, LOW);
  ledcWrite(PWM_Chan2, 0);
  
}
