#include <Servo.h>                                                             // include the servo library
Servo myservo;                                                                 // two servo objects declared for the fans
Servo myservo1;

// Include required libraries for Joystick
#include <avr/wdt.h>                                                          // library required for watchdog
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
/*
const int leftin1 = 33;                                                         // inverted pins
const int leftin2 = 31;
const int leftpwm = 3;

const int rightin1 = 25;
const int rightin2 = 23;
const int rightpwm = 8;

const int fleftin1 = 27;
const int fleftin2 = 29;
const int fleftpwm = 2;

const int frightin1 = 22;
const int frightin2 = 24;
const int frightpwm = 9;
*/
// red team
const int leftin1 = 24;
const int leftin2 = 22;
const int leftpwm = 9;

const int rightin1 = 29;
const int rightin2 = 27;
const int rightpwm = 2;

const int fleftin1 = 23;
const int fleftin2 = 25;
const int fleftpwm = 8;

const int frightin1 = 31;
const int frightin2 = 33;
const int frightpwm = 3;

//Define pin number for Solenoid
//int piston = 47;                                                          // not required for now

// Set the initial pwm for motors
const int lpwm = 255, rpwm = 255, subtract = 155;
int x, y;
const int less = 20, more = 20;                                              // increment decrement of pwm for motors
int  set = 0, set1 = 0;

const int uppin = 15;                                                       // high signal on these pins move the shaft up or down
const int downpin = 16;

const int Pin = 5;                                                            // pins used for two fans
const int Pin1 = 4;
const int buzz_pin = 47;                                                    // pin used for buzzer , buzz when bot is to near to wall

int ultra_sonic_front_echo = 39;                                            // pins for ultrasonic
int ultra_sonic_front_trig = 41;

long duration_front;                                                        // variables for ultrasonic
int distance_front;

bool fan = 0;                                                               // decision variable , blows one fan or both fans
bool debug = 1;

int cfan = 0;

void setup() {
  // Setting the Serial Communication to display data on Serial Monitor
  if (debug == 1) {
    Serial.begin(115200);
  }
  while (!Serial);
  if (debug == 1) {
    Serial.println("Start");
  }
  if (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay(200);

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);

  // Define pinmode for Motors
  pinMode(leftin1, OUTPUT);
  pinMode(leftin2, OUTPUT);
  pinMode(leftpwm, OUTPUT);

  pinMode(rightin1, OUTPUT);
  pinMode(rightin2, OUTPUT);
  pinMode(rightpwm, OUTPUT);

  pinMode(fleftin1, OUTPUT);
  pinMode(fleftin2, OUTPUT);
  pinMode(fleftpwm, OUTPUT);

  pinMode(frightin1, OUTPUT);
  pinMode(frightin2, OUTPUT);
  pinMode(frightpwm, OUTPUT);

  /*
                                                                              // Define pin mode  for Solenoid
  pinMode(piston, OUTPUT);
                                                                              // Initially Solenoid is off
  analogWrite(piston, 0);
  */
  pinMode(uppin, OUTPUT);                                                     // up down shaft pins declared as output
  pinMode(downpin, OUTPUT);

  digitalWrite(uppin, LOW);                                                   // up down pin gives LOW on start
  digitalWrite(downpin, LOW);

  pinMode(buzz_pin, OUTPUT);                                                  /// buzzer pin declared output

  pinMode(Pin, OUTPUT);                                                       // pins for fan declared as output
  analogWrite(Pin, 0);                                                        // pin is given 0 voltage at beginning
  pinMode(Pin1, OUTPUT);
  analogWrite(Pin1, 0);
  myservo.attach(Pin);                                                    // attach first fan to pin 5
  myservo1.attach(Pin1);                                                  // attach the 2 nd fan to pin 4
  myservo.write(20);                                                      // start the edf fan
  myservo1.write(20);

  delay(3000);                                                            // some delay for fan

  pinMode(ultra_sonic_front_trig, OUTPUT);                                // ultrasonic pin declared as output n input
  pinMode(ultra_sonic_front_echo, INPUT);

  wdt_enable(WDTO_1S);
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
  /*                                                                                // code used to buzz the buzzer if to close to the bot
    digitalWrite(ultra_sonic_front_trig, LOW);                                      //  ultrasonic sending signal
    delayMicroseconds(2);
    digitalWrite(ultra_sonic_front_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultra_sonic_front_trig, LOW);

    duration_front = pulseIn(ultra_sonic_front_echo, HIGH);                         // calculating distance in cm
    distance_front = (duration_front / 2) / 29.1;

    if(distance_front > 3){
      digitalWrite(buzz_pin, LOW);
    }
    else if(distance_front <= 3){
      digitalWrite(buzz_pin, LOW);//HIGH);
      if(distance_front == 0) digitalWrite(buzz_pin, LOW);
    }
    Serial.println(distance_front);
  */
}

#include "hidjoystickrptparser.h"

JoystickReportParser::JoystickReportParser(JoystickEvents *evt) :
  joyEvents(evt),
  oldHat(0xDE),
  oldButtons(0) {
  for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
    oldPad[i] = 0xD;
}

void JoystickReportParser::Parse(HID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  bool match = true;

  // Checking if there are changes in report since the method was last called
  for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
    if (buf[i] != oldPad[i]) {
      match = false;
      break;
    }

  // Calling Game Pad event handler
  if (!match && joyEvents) {
    joyEvents->OnGamePadChanged((const GamePadEventData*)buf);

    for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++) oldPad[i] = buf[i];
  }

  uint8_t hat = (buf[5] & 0xF);

  // Calling Hat Switch event handler
  if (hat != oldHat && joyEvents) {
    joyEvents->OnHatSwitch(hat);
    oldHat = hat;
  }

  uint16_t buttons = (0x0000 | buf[6]);
  buttons <<= 4;
  buttons |= (buf[5] >> 4);
  uint16_t changes = (buttons ^ oldButtons);

  // Calling Button Event Handler for every button changed
  if (changes) {
    for (uint8_t i = 0; i < 0x0C; i++) {
      uint16_t mask = (0x0001 << i);

      if (((mask & changes) > 0) && joyEvents)
        if ((buttons & mask) > 0)
          joyEvents->OnButtonDn(i + 1);
        else
          joyEvents->OnButtonUp(i + 1);
    }
    oldButtons = buttons;
  }
}

void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt) {
  if (debug == 1) {
    Serial.print("X1: ");
    PrintHex<uint8_t > (evt->X, 0x80);
    Serial.print("\tY1: ");
    PrintHex<uint8_t > (evt->Y, 0x80);
    Serial.print("\tX2: ");
    PrintHex<uint8_t > (evt->Z1, 0x80);
    Serial.print("\tY2: ");
    PrintHex<uint8_t > (evt->Z2, 0x80);
    Serial.print("\tRz: ");
    PrintHex<uint8_t > (evt->Rz, 0x80);
    Serial.println("");
  }

  int d = evt->Rz;
  d = 128  - d;
  d = map(d, 0, 128, 60, 120);
  if (d == 60) {
    d = 20;
  }
  if (fan == 1) {
    myservo.write(d);
  }
  myservo1.write(d);
  if (debug == 1) {
    Serial.println(d);
    Serial.println("");
  }

  // Non-analog code

  if (evt->Y < 127 && evt->X == 127) {                                      // FORWARD RUN
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);

    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);

    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);

    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);
    //For steady bot movement
    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(fleftpwm, i);
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }
  if (evt->Y > 127 && evt->X == 127) {                                                     //  REVERSE RUN
    digitalWrite(leftin2, HIGH);
    digitalWrite(leftin1, LOW);

    digitalWrite(rightin2, HIGH);
    digitalWrite(rightin1, LOW);

    digitalWrite(fleftin2, HIGH);
    digitalWrite(fleftin1, LOW);

    digitalWrite(frightin2, HIGH);
    digitalWrite(frightin1, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(fleftpwm, i);
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }
  if (evt->X > 127 && evt->Y == 127) {                                               // RIGHT SIDE
    digitalWrite(leftin2, HIGH);
    digitalWrite(leftin1, LOW);

    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);

    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);

    digitalWrite(frightin2, HIGH);
    digitalWrite(frightin1, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(fleftpwm, i);
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }
  if (evt->X < 127 && evt->Y == 127) {                                                //LEFT SIDE
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);

    digitalWrite(rightin2, HIGH);
    digitalWrite(rightin1, LOW);

    digitalWrite(fleftin2, HIGH);
    digitalWrite(fleftin1, LOW);

    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(fleftpwm, i);
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }

  if (evt->X < 127 && evt->Y < 127) {                                     // RIGHT DIAGONAL BACK
    digitalWrite(leftin2, HIGH);
    digitalWrite(leftin1, LOW);
    digitalWrite(frightin2, HIGH);
    digitalWrite(frightin1, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      delay(10);
    }
  }
  if (evt->X > 127 && evt->Y < 127) {                                     //RIGHT DIAGONAL
    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);
    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(fleftpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }
  if (evt->X < 127 && evt->Y < 127) {                                      // LEFT DIAGONAL
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);
    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      delay(10);
    }
  }
  if (evt->X < 127 && evt->Y > 127) {                                             //LEFT DIAGONAL BACK
    digitalWrite(fleftin2, HIGH);
    digitalWrite(fleftin1, LOW);
    digitalWrite(rightin2, HIGH);
    digitalWrite(rightin1, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(fleftpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }

  if (evt->X == 127 && evt->Y == 127) {                                            // TO STOP
    for (int i = lpwm; i >= 0; i = i - less) {
      analogWrite(leftpwm, i);
      analogWrite(rightpwm, i);
      analogWrite(fleftpwm, i);
      analogWrite(frightpwm, i);
      delay(10);
    }
    digitalWrite(leftin1, LOW);
    digitalWrite(leftin2, LOW);

    digitalWrite(rightin1, LOW);
    digitalWrite(rightin2, LOW);

    digitalWrite(fleftin1, LOW);
    digitalWrite(fleftin2, LOW);

    digitalWrite(frightin1, LOW);
    digitalWrite(frightin2, LOW);

    analogWrite(leftpwm, 0);
    analogWrite(rightpwm, 0);
    analogWrite(fleftpwm, 0);
    analogWrite(frightpwm, 0);
  }
  // Non Analog Code end
}

void JoystickEvents::OnHatSwitch(uint8_t hat) {
  if (hat == 00) {                                                  // forward
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);
    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);
    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);
    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(fleftpwm, i);
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }
  if (hat == 04) {                                        // reverse
    digitalWrite(leftin2, HIGH);
    digitalWrite(leftin1, LOW);
    digitalWrite(rightin2, HIGH);
    digitalWrite(rightin1, LOW);
    digitalWrite(fleftin2, HIGH);
    digitalWrite(fleftin1, LOW);
    digitalWrite(frightin2, HIGH);
    digitalWrite(frightin1, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(fleftpwm, i);
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }
  if (hat == 02) {                                              // right side
    digitalWrite(leftin2, HIGH);
    digitalWrite(leftin1, LOW);
    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);
    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);
    digitalWrite(frightin2, HIGH);
    digitalWrite(frightin1, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(fleftpwm, i);
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }
  if (hat == 06) {                                                      // left
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);
    digitalWrite(rightin2, HIGH);
    digitalWrite(rightin1, LOW);
    digitalWrite(fleftin2, HIGH);
    digitalWrite(fleftin1, LOW);
    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(fleftpwm, i);
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }
  if (hat == 03) {                                                        // right diagonal back
    digitalWrite(leftin2, HIGH);
    digitalWrite(leftin1, LOW);
    digitalWrite(frightin2, HIGH);
    digitalWrite(frightin1, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      delay(10);
    }
  }
  if (hat == 01) {                                                    // right diagonal
    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);
    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(fleftpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }
  if (hat == 07) {                                                    // left diagonal
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);
    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      delay(10);
    }
  }
  if (hat == 05) {                                                    // left diagonal back
    digitalWrite(fleftin2, HIGH);
    digitalWrite(fleftin1, LOW);

    digitalWrite(rightin2, HIGH);
    digitalWrite(rightin1, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(fleftpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }
  if (hat == 0x0F) {                                                        // to stop
    digitalWrite(leftin1, LOW);
    digitalWrite(leftin2, LOW);

    digitalWrite(rightin1, LOW);
    digitalWrite(rightin2, LOW);

    digitalWrite(fleftin1, LOW);
    digitalWrite(fleftin2, LOW);

    digitalWrite(frightin1, LOW);
    digitalWrite(frightin2, LOW);

    analogWrite(leftpwm, 0);
    analogWrite(rightpwm, 0);
    analogWrite(fleftpwm, 0);
    analogWrite(frightpwm, 0);
  }
}

void JoystickEvents::OnButtonUp(uint8_t but_id) {
  if (but_id == 1) {                                                                    // shaft goes up
    digitalWrite(uppin, LOW);
  }
  if (but_id == 3) {                                                                    // shaft goes down
    digitalWrite(downpin, LOW);
  }
/*
  if (but_id == 9) {                                                                    // pneumatic goes on
    analogWrite(piston, 0);
  }
*/  
  if (but_id == 10) {

  }
  if ( but_id == 2 || but_id == 4 ||  but_id == 7 || but_id == 8) {                             // To stop
    for (int i = lpwm; i >= 0; i = i - less) {
      analogWrite(leftpwm, i);
      analogWrite(rightpwm, i);
      analogWrite(fleftpwm, i);
      analogWrite(frightpwm, i);
      delay(10);
    }
    digitalWrite(leftin1, LOW);
    digitalWrite(leftin2, LOW);
    digitalWrite(rightin1, LOW);
    digitalWrite(rightin2, LOW);
    digitalWrite(fleftin1, LOW);
    digitalWrite(fleftin2, LOW);
    digitalWrite(frightin1, LOW);
    digitalWrite(frightin2, LOW);

    analogWrite(leftpwm, 0);
    analogWrite(rightpwm, 0);
    analogWrite(fleftpwm, 0);
    analogWrite(frightpwm, 0);
  }
}

void JoystickEvents::OnButtonDn(uint8_t but_id) {
  if (but_id == 5) {
    if (fan == 0) {
      fan = 1;
    }
    else if (fan == 1) {
      fan = 0;
    }
  }
  if (but_id == 1) {
    digitalWrite(uppin, HIGH);
    if (debug == 1) {
      Serial.println("Up pressed");
    }
  }
  if (but_id == 3) {
    digitalWrite(downpin , HIGH);
    if (debug == 1) {
      Serial.println("Down pressed");
    }
  }
  if (but_id == 7) {                                                            // right rotate
    digitalWrite(leftin2, HIGH);
    digitalWrite(leftin1, LOW);

    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);

    digitalWrite(fleftin2, HIGH);
    digitalWrite(fleftin1, LOW);

    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);


    for (int i = 0; i <= (lpwm/* - subtract*/); i = i + /*(2 * */more) {
      analogWrite(fleftpwm, i);
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }

  if (but_id == 8) {                                                                //  left rotate
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);

    digitalWrite(rightin2, HIGH);
    digitalWrite(rightin1, LOW);

    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);

    digitalWrite(frightin2, HIGH);
    digitalWrite(frightin1, LOW);

    for (int i = 0; i <= (lpwm/* - subtract*/); i = i + /*(2 * */more) {
      analogWrite(fleftpwm, i);
      analogWrite(leftpwm, i);
      analogWrite(frightpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }
  if (but_id == 2) {                                                                    //  right turn
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);

    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(fleftpwm, i);
      analogWrite(leftpwm, i);
      delay(10);
    }

  }
  if (but_id == 4) {                                                                      //  left turn
    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);

    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);

    for (int i = 0; i <= lpwm; i = i + more) {
      analogWrite(frightpwm, i);
      analogWrite(rightpwm, i);
      delay(10);
    }
  }
/*
  if (but_id == 9) {
    if (debug == 1) {
      Serial.println("Runned");
    } analogWrite(piston, 255);
  }
  
  if (but_id == 10) {
    if (set1 == 0) {
      Serial.println("Runned 1");
      set1 = 1;
      analogWrite(piston1, 255);
      delay(3000);
      analogWrite(piston1, 0);
    }
  }
  */
  if(but_id == 10){
    cfan++;
    if(cfan == 1){
      myservo.write(80);
      myservo1.write(80);
    }
    else if(cfan == 2){
      myservo.write(90);
      myservo1.write(90);
    }
    else if(cfan == 3){
      myservo.write(100);
      myservo1.write(100);
    }
    else if(cfan == 4){
      myservo.write(110);
      myservo1.write(110);
    }
    else if(cfan == 5){
      myservo.write(120);
      myservo1.write(120);
    }
    else if(cfan == 6){
      myservo.write(20);
      myservo1.write(20);
      cfan = 0;
    }
  }
} 
