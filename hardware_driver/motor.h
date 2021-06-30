// motor.h
#ifndef _MOTOR_H
#define _MOTOR_H

class Motor{
  public:
    Motor(const int IN1, const int IN2, const int EN);
    bool init(void);
    void stop(void);
    bool writeVelocity(int value);
   private:
    int IN1;
    int IN2;
    int EN;
    void forward(int speed);
    void backward(int speed);  
};

#endif
