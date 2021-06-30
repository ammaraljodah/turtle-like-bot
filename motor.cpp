#include "motor.h"
#include "Arduino.h"
Motor::Motor(const int IN1, const int IN2, const int EN){
  this->IN1 = IN1;
  this->IN2 = IN2;
  this->EN = EN;
}
bool Motor::init(void){
  pinMode(EN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  this->stop();
}

void Motor::stop(void){
  analogWrite(EN, 0);
}

bool Motor::writeVelocity(int value){
  if (value>0)
    this->forward(value);
  else
    this->backward(-value);
}

void Motor::forward(int speed){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN, speed);
  
}
void Motor::backward(int speed){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN, speed);
}

   
