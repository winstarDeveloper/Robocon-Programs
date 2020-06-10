#include <PID_v1.h>
double Kp = 5.4, Ki = 0, Kd = 2.4;                    //test
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


#include <Servo.h>
Servo myservo;

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

#include "pins.h"

// Set the initial pwm for motors
const int lpwm = 255, rpwm = 255, subtract = 0;
int x, y;
bool run = 0;
const int less = 20, more = 20;
int  set = 0, set1 = 0;
bool updown = 0, debug = 1;
int count = 0;
int countt = 0;

bool r = 1;                                                                   // decision variable used for starting motors slowly
#define DELAY 10

bool control = 0;                                         // controls the PID wall follower code
const int distance = 18;

int kount = 0;

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

  pinMode(Fpin1, OUTPUT);
  pinMode(Fpin2, OUTPUT);
  pinMode(Fpwm, OUTPUT);

  pinMode(ultra_sonic_front_trig, OUTPUT);
  pinMode(ultra_sonic_front_echo, INPUT);


  // Define pin mode  for Solenoid
  pinMode(piston, OUTPUT);
  //  pinMode(Piston1, OUTPUT);
  // Initially Solenoid is off
  analogWrite(piston, 0);
  //  analogWrite(Piston1, 0);

  pinMode(uppin, OUTPUT);
  pinMode(downpin, OUTPUT);

  digitalWrite(uppin, LOW);
  digitalWrite(downpin, LOW);

  pinMode(Pin, OUTPUT);
  analogWrite(Pin, 0);
  myservo.attach(Pin);
  myservo.write(20);
  delay(3000);
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
  count = 0;


  Input = distance;                                                                         //  setpoint set here - distance from wall
  Setpoint = distance;

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
}

void loop() {
  wdt_reset();
  Usb.Task();

  /************************************Wall Follower***************************************/
  long duration_front, distance_front, duration_back, distance_back;
  if (control == 1)
  {
    digitalWrite(ultra_sonic_front_trig, LOW);                                      //  ultrasonic sending signal
    delayMicroseconds(2);
    digitalWrite(ultra_sonic_front_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultra_sonic_front_trig, LOW);

    duration_front = pulseIn(ultra_sonic_front_echo, HIGH);                         // calculating distance in cm
    distance_front = (duration_front / 2) / 29.1;

    if (debug == 1) {
      Serial.print("Distance front =");
      Serial.println(distance_front);
    }
    Input = distance_front;
    myPID.Compute();                                                                // pid calculates output
    if (debug == 1) {
      Serial.print("Output:");
      Serial.println(Output);
    }
    if (Output == 0)                                                                //  pid controls the bot
    {
      digitalWrite(leftin1, HIGH);
      digitalWrite(leftin2, LOW);

      digitalWrite(rightin1, HIGH);
      digitalWrite(rightin2, LOW);

      digitalWrite(fleftin1, HIGH);
      digitalWrite(fleftin2, LOW);

      digitalWrite(frightin1, HIGH);
      digitalWrite(frightin2, LOW);

      analogWrite(fleftpwm, 255);
      analogWrite(leftpwm, 255);
      analogWrite(frightpwm, 255);
      analogWrite(rightpwm, 255);

    }
    else if (Output > 0)
    {
      digitalWrite(leftin1, HIGH);
      digitalWrite(leftin2, LOW);

      digitalWrite(rightin1, HIGH);
      digitalWrite(rightin2, LOW);

      digitalWrite(fleftin1, HIGH);
      digitalWrite(fleftin2, LOW);

      digitalWrite(frightin1, HIGH);
      digitalWrite(frightin2, LOW);

      if (Output > 127) {
        Output = 127;
      }

      int right = 255;
      int left = 255 - (2 * Output);

      analogWrite(fleftpwm, left);
      analogWrite(leftpwm, left);
      analogWrite(frightpwm, right);
      analogWrite(rightpwm, right);
      if (debug == 1) {
        Serial.print("right =");
        Serial.println(right);
        Serial.print("left =");
        Serial.println(left);
      }
    }
    else if (Output < 0) {
      signed a = (-1 * Output);
      digitalWrite(leftin1, HIGH);
      digitalWrite(leftin2, LOW);

      digitalWrite(rightin1, HIGH);
      digitalWrite(rightin2, LOW);

      digitalWrite(fleftin1, HIGH);
      digitalWrite(fleftin2, LOW);

      digitalWrite(frightin1, HIGH);
      digitalWrite(frightin2, LOW);

      if (a > 127) {
        a = 127;
      }
      int right = 255 - (2 * a);
      int left = 255;

      analogWrite(fleftpwm, left);
      analogWrite(leftpwm, left);
      analogWrite(frightpwm, right);
      analogWrite(rightpwm, right);

      if (debug == 1) {
        Serial.print("right =");
        Serial.println(right);
        Serial.print("left =");
        Serial.println(left);
      }
      delay(50);
    }
  }
  //  ****************************************************************************************

}

#include "hidjoystickrptparser.h"
#include "faltu.h"

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

  int d = evt->Rz;                                                         // get the value from joystick gear
  d = 128  - d;
  d = map(d, 0, 128, 60, 120);
  if (d == 60) {
    d = 20;
  }
  myservo.write(d);                                                        // run the fan
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
    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(leftpwm, i);
        analogWrite(frightpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
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
    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(leftpwm, i);
        analogWrite(frightpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
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

    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(leftpwm, i);
        analogWrite(frightpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
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
    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(leftpwm, i);
        analogWrite(frightpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
    }
  }
  /* Bot hangs up in Analog so removed
      if (evt->X < 127 && evt->Y < 127) {                                     // RIGHT DIAGONAL BACK
        digitalWrite(leftin2, HIGH);
        digitalWrite(leftin1, LOW);
        digitalWrite(frightin2, HIGH);
        digitalWrite(frightin1, LOW);
        if (r == 1) {
          for (int i = 0; i <= lpwm; i = i + more) {
            analogWrite(leftpwm, i);
            analogWrite(frightpwm, i);
            delay(DELAY);
          }
          r = 1;
        }
      }

      if (evt->X > 127 && evt->Y < 127) {                                     //RIGHT DIAGONAL
        digitalWrite(fleftin1, HIGH);
        digitalWrite(fleftin2, LOW);
        digitalWrite(rightin1, HIGH);
        digitalWrite(rightin2, LOW);
          if(r == 1){
          for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(rightpwm, i);
            delay(DELAY);
          }
          r = 1;
          }
      }
      if (evt->X < 127 && evt->Y < 127) {                                      // LEFT DIAGONAL
        digitalWrite(leftin1, HIGH);
        digitalWrite(leftin2, LOW);
        digitalWrite(frightin1, HIGH);
        digitalWrite(frightin2, LOW);
  if (r == 1) {
         for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(leftpwm, i);
        analogWrite(frightpwm, i);
            delay(DELAY);
          }
          r = 1;
          }
      }
      if (evt->X < 127 && evt->Y > 127) {                                             //LEFT DIAGONAL BACK
        digitalWrite(fleftin2, HIGH);
        digitalWrite(fleftin1, LOW);
        digitalWrite(rightin2, HIGH);
        digitalWrite(rightin1, LOW);
  if (r == 1) {
         for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(rightpwm, i);
            delay(DELAY);
         }
      }
      r = 1;
      }
  */
  if (evt->X == 127 && evt->Y == 127) {                                            // TO STOP
    r = 1;
    digitalWrite(leftin1, LOW);
    digitalWrite(leftin2, LOW);

    digitalWrite(rightin1, LOW);
    digitalWrite(rightin2, LOW);

    digitalWrite(fleftin1, LOW);
    digitalWrite(fleftin2, LOW);

    digitalWrite(frightin1, LOW);
    digitalWrite(frightin2, LOW);
    /*
        for (int i = lpwm; i >= 0; i = i - less) {
          analogWrite(leftpwm, i);
          analogWrite(rightpwm, i);
          analogWrite(fleftpwm, i);
          analogWrite(frightpwm, i);
          delay(DELAY);
        }
    */
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
    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(leftpwm, i);
        analogWrite(frightpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
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
    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(leftpwm, i);
        analogWrite(frightpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
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
    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(leftpwm, i);
        analogWrite(frightpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
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
    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(leftpwm, i);
        analogWrite(frightpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
    }
  }
  if (hat == 03) {                                                        // right diagonal back
    digitalWrite(leftin2, HIGH);
    digitalWrite(leftin1, LOW);
    digitalWrite(frightin2, HIGH);
    digitalWrite(frightin1, LOW);
    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(leftpwm, i);
        analogWrite(frightpwm, i);
        delay(DELAY);
      }
      r = 1;
    }
  }
  if (hat == 01) {                                                    // right diagonal
    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);
    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);
    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
    }
  }
  if (hat == 07) {                                                    // left diagonal
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);
    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);
    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(leftpwm, i);
        analogWrite(frightpwm, i);
        delay(DELAY);
      }
      r = 1;
    }
  }
  if (hat == 05) {                                                    // left diagonal back
    digitalWrite(fleftin2, HIGH);
    digitalWrite(fleftin1, LOW);

    digitalWrite(rightin2, HIGH);
    digitalWrite(rightin1, LOW);
    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
    }
  }
  if (hat == 0x0F) {                                                        // to stop
    r = 1;
    digitalWrite(leftin1, LOW);
    digitalWrite(leftin2, LOW);

    digitalWrite(rightin1, LOW);
    digitalWrite(rightin2, LOW);

    digitalWrite(fleftin1, LOW);
    digitalWrite(fleftin2, LOW);

    digitalWrite(frightin1, LOW);
    digitalWrite(frightin2, LOW);
    /*
        for (int i = lpwm; i >= 0; i = i - less) {
          analogWrite(leftpwm, i);
          analogWrite(rightpwm, i);
          analogWrite(fleftpwm, i);
          analogWrite(frightpwm, i);
          delay(DELAY);
        }
    */
    analogWrite(leftpwm, 0);
    analogWrite(rightpwm, 0);
    analogWrite(fleftpwm, 0);
    analogWrite(frightpwm, 0);
  }
}

void JoystickEvents::OnButtonUp(uint8_t but_id) {
  /*  if (but_id == 5) {
      digitalWrite(Fpin1, HIGH);
      digitalWrite(Fpin2, HIGH);
      analogWrite(Fpwm, 255);
    }
    if (but_id == 6) {
      digitalWrite(Fpin1, HIGH);
      digitalWrite(Fpin2, HIGH);
      analogWrite(Fpwm, 255);
    }
    */
  if (but_id == 1) {                                                                // take up the shaft
    digitalWrite(uppin, LOW);
  }
  if (but_id == 3) {                                                                // take down the shaft
    digitalWrite(downpin, LOW);
  }
  if (but_id == 9) {
    //    set = 0;
    analogWrite(piston, 0);
  }
  if (but_id == 10) {
    set1 = 0;
  }
  if ( but_id == 2 || but_id == 4 ||  but_id == 7 || but_id == 8 || but_id == 5 || but_id == 6) {                             // To stop
    r = 1;
    digitalWrite(leftin1, LOW);
    digitalWrite(leftin2, LOW);
    digitalWrite(rightin1, LOW);
    digitalWrite(rightin2, LOW);
    digitalWrite(fleftin1, LOW);
    digitalWrite(fleftin2, LOW);
    digitalWrite(frightin1, LOW);
    digitalWrite(frightin2, LOW);
    /*
        for (int i = lpwm; i >= 0; i = i - less) {
          analogWrite(leftpwm, i);
          analogWrite(rightpwm, i);
          analogWrite(fleftpwm, i);
          analogWrite(frightpwm, i);
          delay(DELAY);
        }
    */
    analogWrite(leftpwm, 0);
    analogWrite(rightpwm, 0);
    analogWrite(fleftpwm, 0);
    analogWrite(frightpwm, 0);
    control = 0;
  }
}

void JoystickEvents::OnButtonDn(uint8_t but_id) {
  if (but_id == 5) {
    /*
    digitalWrite(Fpin2, HIGH);
    digitalWrite(Fpin1, LOW);
    analogWrite(Fpwm, 255);
    */
    /*    // left diagonal
       digitalWrite(leftin1, HIGH);
        digitalWrite(leftin2, LOW);
        digitalWrite(frightin1, HIGH);
        digitalWrite(frightin2, LOW);
        if (r == 1) {
          for (int i = 0; i <= lpwm; i = i + more) {
            analogWrite(leftpwm, i);
            analogWrite(frightpwm, i);
            delay(DELAY);
          }
          r = 1;
        }
    */
    control = 1;
  }

  if (but_id == 6) {
    /*
    digitalWrite(Fpin1, HIGH);
    digitalWrite(Fpin2, LOW);
    analogWrite(Fpwm, 255);
    */
    // right diagonal
    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);
    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);
    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
    }
  }
  if (but_id == 1) {
    // if (updown == 1) {
    digitalWrite(uppin, HIGH);
    Serial.println("Up pressed");
    // }
  }
  if (but_id == 3) {
    //if (updown == 1) {
    digitalWrite(downpin , HIGH);
    Serial.println("Down pressed");
    // }
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

    if (r == 1) {
      for (int i = 0; i <= lpwm/* - subtract)*/; i = i + /*(2 * */ more) {
        analogWrite(fleftpwm, i);
        analogWrite(leftpwm, i);
        analogWrite(frightpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
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
    if (r == 1) {
      for (int i = 0; i <= lpwm/* - subtract)*/; i = i + /*(2 * */more) {
        analogWrite(fleftpwm, i);
        analogWrite(leftpwm, i);
        analogWrite(frightpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
    }
  }
  if (but_id == 2) {                                                                    //  right turn
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);

    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);

    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(fleftpwm, i);
        analogWrite(leftpwm, i);
        delay(DELAY);
      }
      r = 1;
    }
  }
  if (but_id == 4) {                                                                      //  left turn
    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);

    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);
    if (r == 1) {
      for (int i = 0; i <= lpwm; i = i + more) {
        analogWrite(frightpwm, i);
        analogWrite(rightpwm, i);
        delay(DELAY);
      }
      r = 1;
    }
  }
  if (but_id == 9) {
    /*  //    if (set == 0) {
      Serial.println("Runned");
      //      set = 1;
      analogWrite(piston, 255);
      //      delay(4500);
      //      analogWrite(piston, 0);
      //    }
    */
    countt++;
    if (countt == 1) {
      myservo.write(80);
    }
    else if (countt == 2) {
      myservo.write(100);
    }
    else if (countt == 3) {
      myservo.write(120);
    }
    else if (countt == 4) {
      myservo.write(20);
      countt  = 0;
    }
  }
  if (but_id == 10) {
    count++;
    if (count == 1) {
      myservo.write(80);
    }
    else if (count == 2) {
      myservo.write(90);
    }
    else if (count == 3) {
      myservo.write(100);
    }
    else if (count == 4) {
      myservo.write(110);
    }
    else if (count == 5) {
      myservo.write(120);
    }
    else if (count == 6) {
      myservo.write(20);
      count = 0;
    }
  }
  /*
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


  if (but_id == 7) {
    kount++;
    if (kount > 2) {
      kount = 0;
    }
    myPID.SetTunings(Kp, Ki, Kd);
  }
  if (but_id == 6) {
    if (kount == 0) {
      Kp = Kp + 0.5;
    }
    else if (kount == 1) {
      Ki = Ki + 0.01;
    }
    else if (kount == 2) {
      Kd = Kd + 0.1;
    }
  }

  if (but_id == 8) {
    if (kount == 0) {
      Kp = Kp - 0.5;
    }
    else if (kount == 1) {
      Ki = Ki - 0.01;
    }
    else if (kount == 2) {
      Kd = Kd - 0.1;
    }
  }
}
