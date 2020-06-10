// Include required libraries for Joystick
#include <avr/wdt.h>
#include <hid.h>
#include <hiduniversal.h                                    >
#include <usbhub.h>
#include "hidjoystickrptparser.h"

// Declare USB class variable
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

// Define Pin Number for Motors
const int Leftin1 = 31;
const int Leftin2 = 33;
const int Leftpwm = 3;

const int Rightin1 = 23;
const int Rightin2 = 25;
const int Rightpwm = 8;

const int Fleftin1 = 27;
const int Fleftin2 = 29;
const int Fleftpwm = 2;

const int Frightin1 = 24;
const int Frightin2 = 22;
const int Frightpwm = 9;

// Define pin for Solenoid
const int Piston = 47;
//const int Piston1 = 0;

const int Uppin = 15;
const int Downpin = 16;

const int fpin1 = 37;
const int fpin2 = 35;
const int fpwm = 4;

void setup() {
  // Setting the Serial Communication to display data on Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Start");

  if (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay(200);

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);

  // Define pinmode for Motors
  pinMode(Leftin1, OUTPUT);
  pinMode(Leftin2, OUTPUT);
  pinMode(Leftpwm, OUTPUT);

  pinMode(Rightin1, OUTPUT);
  pinMode(Rightin2, OUTPUT);
  pinMode(Rightpwm, OUTPUT);

  pinMode(Fleftin1, OUTPUT);
  pinMode(Fleftin2, OUTPUT);
  pinMode(Fleftpwm, OUTPUT);

  pinMode(Frightin1, OUTPUT);
  pinMode(Frightin2, OUTPUT);
  pinMode(Frightpwm, OUTPUT);

  pinMode(fpin1, OUTPUT);
  pinMode(fpin2, OUTPUT);
  pinMode(fpwm, OUTPUT);

  // Define pin mode  for Solenoid
  pinMode(Piston, OUTPUT);
//  pinMode(Piston1, OUTPUT);
  // Initially Solenoid is off
  analogWrite(Piston, 0);
//  analogWrite(Piston1, 0);

  pinMode(Uppin, OUTPUT);
  pinMode(Downpin, OUTPUT);
  
  digitalWrite(Uppin, LOW);
  digitalWrite(Downpin, LOW);

  wdt_enable(WDTO_8S);
/*
Threshold value Constant name Supported on
15 ms WDTO_15MS ATMega 8, 168, 328, 1280, 2560
30 ms WDTO_30MS ATMega 8, 168, 328, 1280, 2560
60 ms WDTO_60MS ATMega 8, 168, 328, 1280, 2560
120 ms  WDTO_120MS  ATMega 8, 168, 328, 1280, 2560
250 ms  WDTO_250MS  ATMega 8, 168, 328, 1280, 2560
500 ms  WDTO_500MS  ATMega 8, 168, 328, 1280, 2560
1 s WDTO_1S ATMega 8, 168, 328, 1280, 2560
2 s WDTO_2S ATMega 8, 168, 328, 1280, 2560
4 s WDTO_4S ATMega 168, 328, 1280, 2560
8 s WDTO_8S ATMega 168, 328, 1280, 2560
 */
}

void loop() {
  wdt_reset();
  Usb.Task();
}
