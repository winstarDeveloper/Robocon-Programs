// Define pin number for Motors
/*
const int leftin1 = 23;
const int leftin2 = 25;
const int leftpwm = 2;

const int rightin1 = 45;
const int rightin2 = 43 ;
const int rightpwm = 8;

const int fleftin1 = 35;
const int fleftin2 = 37;
const int fleftpwm = 5;

const int frightin1 = 39;
const int frightin2 = 41;
const int frightpwm = 6;
*/

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
int piston = 47;// piston1 = 0;
// Set the initial pwm for motors
const int lpwm = 255, rpwm = 255, subtract = 155;
int x, y;
bool run = 0;
const int less = 20, more = 20;
int  set = 0, set1 = 0;
bool updown = 0;
const int uppin = 15;
const int downpin = 16;

const int Fpin1 = 37;
const int Fpin2 = 35;
const int Fpwm = 4;

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
  /*          Serial.print("X1: ");
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
  */
  /*  if (evt->X == 127 && evt->Y == 127) {
      run = 0;
    }
    else if (evt->X == 80 && evt->Y == 80) {
      run = 1;
    }

  */
  //  if (run == 0) {
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

void JoystickEvents::OnButtonUp(uint8_t but_id) {
  if (but_id == 5) {
    digitalWrite(Fpin1, HIGH);
    digitalWrite(Fpin2, HIGH);
    analogWrite(Fpwm, 255);
  }
  if (but_id == 6) {
    digitalWrite(Fpin1, HIGH);
    digitalWrite(Fpin2, HIGH);
    analogWrite(Fpwm, 255);
  }
  if (but_id == 1) {
    digitalWrite(uppin, LOW);
  }
  if (but_id == 3) {
    digitalWrite(downpin, LOW);
  }
  if (but_id == 9) {
//    set = 0;
    analogWrite(piston, 0);
  }
  if (but_id == 10) {
    set1 = 0;
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
  if(but_id == 5){
    digitalWrite(Fpin2, HIGH);
    digitalWrite(Fpin1, LOW);
    analogWrite(Fpwm,255);
  }
  if(but_id == 6){
    digitalWrite(Fpin1, HIGH);
    digitalWrite(Fpin2, LOW);
    analogWrite(Fpwm,255);
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
  if (but_id == 9) {
    //    if (set == 0) {
    Serial.println("Runned");
    //      set = 1;
    analogWrite(piston, 255);
    //      delay(4500);
    //      analogWrite(piston, 0);
    //    }
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
}
