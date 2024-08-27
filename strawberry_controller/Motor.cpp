#include "Motor.h"


Motor::Motor(int out1, int out2, int pwm) {
      m_output1 = out1;
      m_output2 = out2;
      m_pwm.attach(pwm);

      pinMode(m_output1, OUTPUT);
      pinMode(m_output2, OUTPUT);
    }

    void Motor::driveForwardFast() {
      driveForward(FAST);
    }

    void Motor::driveForwardSlow() {
      driveForward(SLOW);
    }

    void Motor::driveReverseFast() {
      driveReverse(FAST);
    }

    void Motor::driveReverseSlow() {
      driveReverse(SLOW);
    }

    /**
       Takes a "speed" between 0 and 180
    */
    void Motor::driveForward(int spd) {
      digitalWrite(m_output1, HIGH);
      digitalWrite(m_output2, LOW);
      m_pwm.write(spd);
    }

    /**
       Takes a "speed" between 0 and 180
    */
    void Motor::driveReverse(int spd) {
      digitalWrite(m_output1, LOW);
      digitalWrite(m_output2, HIGH);
      m_pwm.write(spd);
    }

    void Motor::driveOff() {
      digitalWrite(m_output1, LOW);
      digitalWrite(m_output2, LOW);
    }
