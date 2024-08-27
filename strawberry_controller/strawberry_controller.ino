/***
   Controller code for a robot with two powered wheels with an Adafruit
   Feather 32u4.

   This code is modified from example code provided by Adafruit that
   reads button press packets from the Bluefruit Connect App.
*/

/*********************************************************************
  This is an example for our nRF51822 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
  SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

  Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

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
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
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


  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then select the game controller"));
  Serial.println();

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

  Serial.println(F("******************************"));

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

  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }

    if (!pressed) {

      leftMotor.driveOff();
      rightMotor.driveOff();

    } else if (buttnum == FWD_BUTTON) {
      
      leftMotor.driveForward(TOP_SPEED);
      rightMotor.driveForward(TOP_SPEED);
      
    } else if (buttnum == REV_BUTTON) {
      
      leftMotor.driveReverse(TOP_SPEED);
      rightMotor.driveReverse(TOP_SPEED);
      
    } else if (buttnum == RIGHT_BUTTON) {
      
      leftMotor.driveForward(LOW_SPEED);
      rightMotor.driveReverse(LOW_SPEED);
      
    } else if (buttnum == LEFT_BUTTON) {
      
      leftMotor.driveReverse(LOW_SPEED);
      rightMotor.driveForward(LOW_SPEED);
    }
  }
}
