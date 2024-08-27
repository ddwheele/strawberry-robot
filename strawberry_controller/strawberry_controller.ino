/***
   Controller code for a robot with two powered wheels with an Adafruit
   Feather 32u4.

   This code is modified from example code provided by Adafruit that
   reads button press packets from the Bluefruit Connect App.
*/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include "packetParser.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// the packet buffer
extern uint8_t packetBuffer[];

class Motor {
    int m_output1; // number of pin connected to IN1 on motor controller
    int m_output2; // number of pin connected to IN2 on motor controller
    Servo m_pwm; // number of pin connected to PWM on motor controller

  public:
    Motor(int out1, int out2, int pwm) {
      m_output1 = out1;
      m_output2 = out2;
      m_pwm.attach(pwm);

      pinMode(m_output1, OUTPUT);
      pinMode(m_output2, OUTPUT);
    }

    /**
       Takes a "speed" between 0 and 180
    */
    void driveForward(int spd) {
      digitalWrite(m_output1, HIGH);
      digitalWrite(m_output2, LOW);
      m_pwm.write(spd);
    }

    /**
       Takes a "speed" between 0 and 180
    */
    void driveReverse(int spd) {
      digitalWrite(m_output1, LOW);
      digitalWrite(m_output2, HIGH);
      m_pwm.write(spd);
    }

    void driveOff() {
      digitalWrite(m_output1, LOW);
      digitalWrite(m_output2, LOW);
    }
};

Motor leftMotor(18, 19, 11);
Motor rightMotor(20, 21, 12);

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  initialize();
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/

#define FWD_BUTTON 5
#define REV_BUTTON 6
#define LEFT_BUTTON 7
#define RIGHT_BUTTON 8

#define TOP_SPEED 180
#define LOW_SPEED 90

void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  // Buttons
  if (packetBuffer[1] == 'B') {
    uint8_t buttonNumber = packetBuffer[2] - '0';
    boolean pressed = packetBuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttonNumber);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }

    if (!pressed) {

      leftMotor.driveOff();
      rightMotor.driveOff();

    } else if (buttonNumber == FWD_BUTTON) {
      
      leftMotor.driveForward(TOP_SPEED);
      rightMotor.driveForward(TOP_SPEED);
      
    } else if (buttonNumber == REV_BUTTON) {
      
      leftMotor.driveReverse(TOP_SPEED);
      rightMotor.driveReverse(TOP_SPEED);
      
    } else if (buttonNumber == RIGHT_BUTTON) {
      
      leftMotor.driveForward(LOW_SPEED);
      rightMotor.driveReverse(LOW_SPEED);
      
    } else if (buttonNumber == LEFT_BUTTON) {
      
      leftMotor.driveReverse(LOW_SPEED);
      rightMotor.driveForward(LOW_SPEED);
    }
  }
}

void initialize() {
   
  delay(500);

  Serial.begin(115200);

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    Error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      Error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("******************************"));
  Serial.println(F("Strawberry Controller"));
  Serial.println(F("******************************"));

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("Ready to drive!"));
}

// A small helper
inline void Error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
