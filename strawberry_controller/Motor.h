#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h>
#include <Arduino.h>

class Motor {
    static const int FAST = 180;
    static const int SLOW = 90;
    int m_output1; // number of pin connected to IN1 on motor controller
    int m_output2; // number of pin connected to IN2 on motor controller
    Servo m_pwm; // number of pin connected to PWM on motor controller

  public:
    Motor(int out1, int out2, int pwm);

    void driveForwardFast();

    void driveForwardSlow();

    void driveReverseFast();

    void driveReverseSlow();

    /**
       Takes a "speed" between 0 and 180
    */
    void driveForward(int spd);

    /**
       Takes a "speed" between 0 and 180
    */
    void driveReverse(int spd);

    void driveOff();
};
#endif
