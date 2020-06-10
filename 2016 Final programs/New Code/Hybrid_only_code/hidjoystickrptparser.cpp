const int leftin1 = 25;
const int leftin2 = 23;
const int leftpwm = 2;

const int rightin1 = 45;
const int rightin2 = 43 ;
const int rightpwm = 8;

const int fleftin1 = 37;
const int fleftin2 = 35;
const int fleftpwm = 5;

const int frightin1 = 33;
const int frightin2 = 31;
const int frightpwm = 4;

const int lpwm = 255,rpwm = 255,subtract = 155;
int x , y;

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
/*  Serial.print("X1: ");
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
  x = evt->X; y = evt->Y;
  
  if (y < 127 && x == 127) {                                      // FORWARD
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);

    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);

    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);

    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);

    analogWrite(fleftpwm, lpwm);
    analogWrite(leftpwm, lpwm);
    delay(17);                                                                // Delay  to go straight
    analogWrite(frightpwm, rpwm);
    delay(33);
    analogWrite(rightpwm, rpwm);
  }
  if (y > 127 && x == 127) {                                                     //  reverse run
    digitalWrite(leftin2, HIGH);
    digitalWrite(leftin1, LOW);

    digitalWrite(rightin2, HIGH);
    digitalWrite(rightin1, LOW);

    digitalWrite(fleftin2, HIGH);
    digitalWrite(fleftin1, LOW);

    digitalWrite(frightin2, HIGH);
    digitalWrite(frightin1, LOW);

    analogWrite(fleftpwm, lpwm);
    analogWrite(leftpwm, lpwm);
    delay(17);                                                                       // Delay to go straight
    analogWrite(frightpwm, rpwm);
    delay(33);
    analogWrite(rightpwm, rpwm);
  }
  if (x > 127 && y == 127) {                                               // right side
    digitalWrite(leftin2, HIGH);
    digitalWrite(leftin1, LOW);

    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);

    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);

    digitalWrite(frightin2, HIGH);
    digitalWrite(frightin1, LOW);

    analogWrite(fleftpwm, lpwm);
    analogWrite(frightpwm, rpwm);

    analogWrite(leftpwm, lpwm);
    analogWrite(rightpwm, rpwm);
  }
  if (x < 127 && y == 127) {                                                // left side
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);

    digitalWrite(rightin2, HIGH);
    digitalWrite(rightin1, LOW);

    digitalWrite(fleftin2, HIGH);
    digitalWrite(fleftin1, LOW);

    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);

    analogWrite(fleftpwm, lpwm);
    analogWrite(frightpwm, rpwm);

    analogWrite(leftpwm, lpwm);
    analogWrite(rightpwm, rpwm);
  }

  if (x < 127 && y < 127) {                                     //  right diagonal back
    digitalWrite(leftin2, HIGH);
    digitalWrite(leftin1, LOW);

    digitalWrite(frightin2, HIGH);
    digitalWrite(frightin1, LOW);

    analogWrite(leftpwm, lpwm);
    analogWrite(frightpwm, rpwm);
  }
  if (x > 127 && y < 127) {                                     //  right diagonal
    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);

    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);

    analogWrite(fleftpwm, lpwm);
    analogWrite(rightpwm, rpwm);
  }
  if (x < 127 && y < 127) {                                      // left diagonal
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);

    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);

    analogWrite(leftpwm, lpwm);
    analogWrite(frightpwm, rpwm);
  }
  if (x < 127 && y > 127) {                                             // left diagonal back
    digitalWrite(fleftin2, HIGH);
    digitalWrite(fleftin1, LOW);

    digitalWrite(rightin2, HIGH);
    digitalWrite(rightin1, LOW);

    analogWrite(fleftpwm, lpwm);
    analogWrite(rightpwm, rpwm);
  }

  if (x == 127 && y == 127) {                                            // to stop
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, HIGH);

    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, HIGH);

    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, HIGH);

    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, HIGH);

    analogWrite(leftpwm, 0);
    analogWrite(rightpwm, 0);
    analogWrite(fleftpwm, 0);
    analogWrite(frightpwm, 0);
  }
}

void JoystickEvents::OnHatSwitch(uint8_t hat) {
/*  Serial.print("Hat Switch: ");
  PrintHex<uint8_t > (hat, 0x80);
  Serial.println("");
*/
}

void JoystickEvents::OnButtonUp(uint8_t but_id) {
/*  Serial.print("Up: ");
  Serial.println(but_id, DEC);
*/

  if ( but_id == 2 || but_id == 4 ||  but_id == 7 || but_id == 8) {                             // To stop
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, HIGH);

    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, HIGH);

    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, HIGH);

    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, HIGH);

    analogWrite(leftpwm, 0);
    analogWrite(rightpwm, 0);
    analogWrite(fleftpwm, 0);
    analogWrite(frightpwm, 0);
  }
}

void JoystickEvents::OnButtonDn(uint8_t but_id) {
/*  Serial.print("Dn: ");
  Serial.println(but_id, DEC);
*/

  if (but_id == 7) {                                                            // right rotate
    digitalWrite(leftin2, HIGH);
    digitalWrite(leftin1, LOW);

    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);

    digitalWrite(fleftin2, HIGH);
    digitalWrite(fleftin1, LOW);

    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);

    analogWrite(fleftpwm, lpwm - subtract);                       // -55 Added as it is going very fast
    analogWrite(frightpwm, rpwm - subtract);

    analogWrite(leftpwm, lpwm - subtract);
    analogWrite(rightpwm, rpwm - subtract);
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

    analogWrite(fleftpwm, lpwm - subtract);                 // -55 Added as it is moving very fast
    analogWrite(frightpwm, rpwm - subtract);

    analogWrite(leftpwm, lpwm - subtract);
    analogWrite(rightpwm, rpwm - subtract);
  }
  if (but_id == 2) {                                                                    //  right turn
    digitalWrite(leftin1, HIGH);
    digitalWrite(leftin2, LOW);

    //   digitalWrite(rightin1, HIGH);
    //   digitalWrite(rightin2, LOW);

    digitalWrite(fleftin1, HIGH);
    digitalWrite(fleftin2, LOW);

    //   digitalWrite(frightin1, HIGH);
    //   digitalWrite(frightin2, LOW);

    analogWrite(leftpwm, lpwm);
    //   analogWrite(rightpwm, rpwm - 30);
    analogWrite(fleftpwm, lpwm);
    //   analogWrite(frightpwm, rpwm - 30);
  }
  if (but_id == 4) {                                                                      //  left turn
    //   digitalWrite(leftin1, HIGH);
    //   digitalWrite(leftin2, LOW);

    digitalWrite(rightin1, HIGH);
    digitalWrite(rightin2, LOW);

    //   digitalWrite(fleftin1, HIGH);
    //   digitalWrite(fleftin2, LOW);

    digitalWrite(frightin1, HIGH);
    digitalWrite(frightin2, LOW);

    //   analogWrite(leftpwm, lpwm - 30);
    analogWrite(rightpwm, rpwm);
    //   analogWrite(fleftpwm, lpwm - 30);
    analogWrite(frightpwm, rpwm);
  }
}
