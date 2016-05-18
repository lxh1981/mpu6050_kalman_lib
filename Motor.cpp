#include "Motor.h"

Motor::Motor(){
	this->enableA_pin = PF_2;
	this->enableB_pin = PF_3;
	this->a1 = PB_3;
	this->a2 = PC_4;
	this->b1 = PC_5;
	this->b2 = PC_6;
	this->distance = 0;
	pinMode(this->enableA_pin,OUTPUT);
	pinMode(this->enableB_pin,OUTPUT);
	pinMode(this->a1,OUTPUT);
	pinMode(this->a2,OUTPUT);
	pinMode(this->b1,OUTPUT);
	pinMode(this->b2,OUTPUT);
        digitalWrite(this->a1,LOW);
        digitalWrite(this->a2,LOW);
        digitalWrite(this->b1,LOW);
        digitalWrite(this->b2,LOW);
        

	
}
Motor::Motor(MotorInit * motorInit){
	this->enableA_pin = motorInit->enA_pin;
	this->enableA_pin = motorInit->enA_pin;
	this->a1 = motorInit->a1;
	this->a2 = motorInit->a2;
	this->b1 = motorInit->b1;
	this->b2 = motorInit->b2;
	this->distance = 0;
	pinMode(this->enableA_pin,OUTPUT);
	pinMode(this->enableB_pin,OUTPUT);
	pinMode(this->a1,OUTPUT);
	pinMode(this->a2,OUTPUT);
	pinMode(this->b1,OUTPUT);
	pinMode(this->b2,OUTPUT);
}
void Motor::control(int out){
	if(out > 0){
		digitalWrite(this->a1,LOW);
		digitalWrite(this->a2,HIGH);
		digitalWrite(this->b1,HIGH);
		digitalWrite(this->b2,LOW);
		
		
	}
	else{
  
                

                digitalWrite(this->a1,HIGH);
		digitalWrite(this->a2,LOW);
		digitalWrite(this->b1,LOW);
		digitalWrite(this->b2,HIGH);


		
	}
	unsigned char vel = abs(out);    // Absolute value of velocity
	analogWrite(this->enableA_pin,vel);
	analogWrite(this->enableB_pin,vel);
}
void Motor::moveForward(unsigned char _speed){
	if(_speed < 0)  _speed = 0;
	if(_speed > 255)  _speed = 255;
	digitalWrite(this->a1,HIGH);
	digitalWrite(this->a2,LOW);
	digitalWrite(this->b1,LOW);
	digitalWrite(this->b2,HIGH);
	analogWrite(this->enableA_pin ,_speed);
	analogWrite(this->enableB_pin, _speed);
	
}
void Motor::moveBackward(unsigned char _speed){
	if(_speed < 0) _speed = 0;
	if(_speed > 255) _speed = 255;
	digitalWrite(this->a1,LOW);
	digitalWrite(this->a2,HIGH);
	digitalWrite(this->b1,HIGH);
	digitalWrite(this->b2,LOW);
	analogWrite(this->enableA_pin ,_speed);
	analogWrite(this->enableB_pin, _speed);
	
}
