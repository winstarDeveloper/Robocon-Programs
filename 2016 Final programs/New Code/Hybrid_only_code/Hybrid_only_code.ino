#include <hid.h>
#include <hiduniversal.h>
#include <usbhub.h>

#include "hidjoystickrptparser.h"

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

const int Leftin1 = 25;
const int Leftin2 = 23;
const int Leftpwm = 2;

const int Rightin1 = 45;
const int Rightin2 = 43 ;
const int Rightpwm = 8;

const int Fleftin1 = 37;
const int Fleftin2 = 35;
const int Fleftpwm = 5;

const int Frightin1 = 33;
const int Frightin2 = 31;
const int Frightpwm = 4;


void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  Serial.println("Start");

  if (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay(200);

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);

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
}

void loop() {
  Usb.Task();
}

