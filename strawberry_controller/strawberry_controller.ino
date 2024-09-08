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
#include "Motor.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

#define FWD_BUTTON 5
#define REV_BUTTON 6
#define LEFT_BUTTON 7
#define RIGHT_BUTTON 8

// Create the bluefruit object
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// the packet buffer
extern uint8_t packetBuffer[];

Motor leftMotor(18, 19, 11); // (A0, A1, 11)
Motor rightMotor(20, 21, 12); // (A2, A3, 12)

void setup(void)
{
  delay(500);

  Serial.begin(115200);

  initializeBluefruit();

  doFactoryReset();

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  // printBluefruitInfo();

  Serial.println(F("******************************"));
  Serial.println(F("Strawberry Controller"));
  Serial.println(F("******************************"));

  waitForConnection();
}

void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  // Buttons
  if (packetBuffer[1] == 'B') {
    uint8_t buttonNumber = packetBuffer[2] - '0';
    boolean pressed = packetBuffer[3] - '0';

    printButtonPress(buttonNumber, pressed);
    executeButtonPress(buttonNumber, pressed);
  }
}

void executeButtonPress(uint8_t buttonNumber, boolean pressed) {
  if (!pressed) {

    leftMotor.driveOff();
    rightMotor.driveOff();

  } else if (buttonNumber == FWD_BUTTON) {

    leftMotor.driveForwardFast();
    rightMotor.driveForwardFast();

  } else if (buttonNumber == REV_BUTTON) {

    leftMotor.driveReverseFast();
    rightMotor.driveReverseFast();

  } else if (buttonNumber == RIGHT_BUTTON) {

    leftMotor.driveForwardSlow();
    rightMotor.driveReverseSlow();

  } else if (buttonNumber == LEFT_BUTTON) {

    leftMotor.driveReverseSlow();
    rightMotor.driveForwardSlow();
  }
}

void printButtonPress(uint8_t buttonNumber, boolean pressed) {
  Serial.print ("Button "); Serial.print(buttonNumber);
  if (pressed) {
    Serial.println(" pressed");
  } else {
    Serial.println(" released");
  }
}

void initializeBluefruit() {
  /* Initialize the module */
  Serial.print(F("Initializing the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    Error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );
}

void doFactoryReset() {
  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      Error(F("Couldn't factory reset"));
    }
  }
}

void waitForConnection() {
  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
  Serial.println(F("========================="));
  Serial.println(F("Ready to drive!"));
}

void printBluefruitInfo() {
  /* Print Bluefruit information */
  Serial.println("Requesting Bluefruit info:");

  ble.info();
  ble.verbose(false);  // debug info is a little annoying after this point!
}

// A small helper
inline void Error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
