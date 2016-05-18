#ifndef MOROTOR_H
#define MOTOR_H
#include <Energia.h> 
  typedef struct x{
    int enA_pin;
    int enB_pin;
    int a1;
    int a2;
    int b1;
    int b2;
  
  }MotorInit;
  class Motor{
	int enableA_pin;
	int enableB_pin;
	int a1;
	int a2;
	int b1;
	int b2;
	double distance;
  public:
	Motor(MotorInit *motorInit);
	Motor();
	void control(int out);
        void moveForward(unsigned char _speed);
        void moveBackward(unsigned char _speed);

  };

#endif

