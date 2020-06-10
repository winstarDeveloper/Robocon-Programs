#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusincludeṁ
#include <spi4teensy3.h>
#include <SPI.h>
#endif

#include "hidjoystickrptparser.h"

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

//stepper pins
/*const int in1=A8;
  const int in2=A9;
  const int in3=A10;
  const int in4=A11;*/
const int dir = A9;
const int pull = A8;

//const int button=A0;
/*
  const int input1=A15;
  const int input2=A14;
  const int input3=A12;
  const int input4=A13;
*/
/*
  const int input1=A8;
  const int input2=A10;
  const int input3=A11;
  const int input4=A13;
*/
const int input1 = A10;//A8 ; //A10;
const int input2 = A9;
const int input3 = A11;
const int input4 = A11; /*...............relay for upper bat service ..................*/

/*..........motor pins..............*/
/*const int pwm1=9;
  const int pwm2=8;
   const int pwm3=6;
   const int pwm4=5;
    const  int motor1in1=47;
   const  int motor1in2=49;
   const  int motor2in1=43;
   const  int motor2in2=45;
   const   int motor3in1=39;
   const  int motor3in2=41;
  const   int motor4in1=35;
   const  int motor4in2=37;
*/
const  int motor1in1 = 41;
const  int motor1in2 = 39;
const  int motor2in1 = 49;
const  int motor2in2 = 47;
const   int motor3in1 = 45;
const  int motor3in2 = 43;
const   int motor4in1 = 37;
const  int motor4in2 = 35;
const int pwm1 = 6;
const int pwm2 = 9;
const int pwm3 = 8;
const int pwm4 = 5;

void setup() {
  pinMode(pwm1, OUTPUT);
  pinMode(motor1in1, OUTPUT);
  pinMode(motor1in2, OUTPUT);

  pinMode(pwm2, OUTPUT);
  pinMode(motor2in1, OUTPUT);
  pinMode(motor2in2, OUTPUT);

  pinMode(pwm3, OUTPUT);
  pinMode(motor3in1, OUTPUT);
  pinMode(motor3in2, OUTPUT);

  pinMode(pwm4, OUTPUT);
  pinMode(motor4in1, OUTPUT);
  pinMode(motor4in2, OUTPUT);
  //  pinMode(dir, OUTPUT);
  // pinMode(pull, OUTPUT);
  // pinMode(button, INPUT);
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);
  /* pinMode(input3, OUTPUT);
    pinMode(input4, OUTPUT);*/
  //digitalWrite(button,LOW);
  digitalWrite(input1, HIGH);
  digitalWrite(input2, HIGH);
  digitalWrite(input3, HIGH);
  digitalWrite(input4, HIGH);

  //stepper operation
  /* pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
     pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
  */
  pinMode(dir, OUTPUT);
  pinMode(pull, OUTPUT);

  digitalWrite(dir, HIGH);
  /* pinMode(input3, HIGH);
    pinMode(input4, HIGH);*/
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  Serial.println("Start");

  if (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay(200);

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);
}

void loop() {
  Usb.Task();
}

