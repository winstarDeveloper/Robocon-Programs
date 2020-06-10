#include "setup.h"

// pan on encoder - done
// decrement disc loader rotation by 1 deg - tried but failed - done
// upper bot off : left right spot, upper bot on : fine adjustment on count - done
// motor started display on lcd - done
// display rmcs angle position - done
// put line follower mode in lcd - done
// shifted motor rpm pid to rpm counter arduino, slow serial so do not press button quickly - done
// set pan on interrupts by ir - setting here - done
// 124 * 50 ... count to load disc - ok
// 68 * 50 = 3250 required for 7 discs - ok
// 11 * 50 for one disc - ok
// try changing serial1 to software serial with rpm arduino - pcd changed no need
// check discload limit switch - no support to this
// remove rpm uncertainty - done

// added disc push motor security feature - add for reverse - done
// print everything on lcd - rpm and pwm , loading motor state, rmcs pushing in or out - done test it
// rmcs encoder values for angle - try again may be possible
// automatic pan and angle set for a given spot - do when the bot is in running - a special mode for this thing - done - test it
// display auto mode on lcd - done - test it
// motor runs continuosly even after upperbotcontrol is off, see to it - remove mode control if possible
// set rotational values for disc loading - needs attention
// perfect disc loading somehow - little bit remaining - try adding a limit switch
// angle set on rheo wire - trying - problem hai
// try pan on new encoder - done - test on arena
// make program for other side
// robot self test at beginning
// stop bot at junction
// change speed to go to loading area
//change angle by rheostat values
// add watchdog to semi automatic - done - test it
// stop bot at junction with count - done - try it
// reverse disc load

// stop the bot if wire is disconnected

// SERIAL COMMUNICATION
// Serial 1 : send receive rpm
// Serial 2 : rmcs for angle
// Serial 3 : rmcs for disc load
// mySerial : communicate with chasis
// BTserial : bluetooth serial module

#define DEBUG 1                         // Serial print on or off in loop()


// PIN Numbers
#define angleMot_1 27
#define angleMot_2 29
#define anglePwm 5

#define panMot_1 11
#define panMot_2 13
#define panPwm 12

#define discPullMot_1 23
#define discPullMot_2 25
#define discPullPwm 4

#define panInterruptPin 21

#define discMot_1 33
#define discMot_2 31
#define discPwm 6

#define startKillSwitch 39  // grey white
#define endKillSwitch 41   // purple blue

#define anglePot A15

//#define discloadRmcsReset 49

#define discloadLimitSwt 43 // green yellow

#define led_1 32
#define led_2 34
#define led_3 36
//#define led_4 22

#include <SoftwareSerial.h>
SoftwareSerial mySerial(7, 35); // RX, TX                        // for communication
SoftwareSerial BTserial(28, 26); // RX | TX                       // for bluetooth

// variables with values
unsigned int panSpeed = 180;
double desired_rpm = 3520;
unsigned int rpmIncrementVal = 10;
String rmcsAngleSpeed = "S-100", rmcsAngleSpeedN = "S160";
static unsigned long discPushWaitTime = 1000, discWaitTime, discloadStartTime, discloadEndTime, throwingStarted, discloadingStartTime;
unsigned int angleVal = 0, velocity = 0, srpm = 0;
int command = 1, rmcsPos;
String str = ":";
unsigned int loadingMotorState = 0;
int autoAdjustCount = -1, autoModeCount = 0;
unsigned int x, y;

int autoValues[8][3] = { {0, 0, 0},
  {3340, 75, 90},  // Spot 1
  {3480, 70, 95},  // Spot 2
  {2500, 70, 100},  // Spot 3
  {3520, 70, 95},  // Spot 4
  {4300, 70, 95},  // Spot 5
  {3480, 70, 95},  // Spot 6
  {3340, 70, 95}   // Spot 7
};
//RPM , ANGLE, PAN

//boolean values
boolean buttonsUP = false, buttonsDOWN = false;
boolean buttonsLEFT = false, buttonsRIGHT = false;              // for Joystick pressed loop operation
boolean lineON = true;                                         // decide wheather line follower on or off and invert mode
boolean stopControl = false;                             // stopControl for bluetooth disconnection control
boolean which = false;
boolean angleButtonPressed = false, angleButtonPressedT = false;
boolean throwingMotor = false;
boolean loadingON = false;
boolean reachedEnd = false;
boolean buttonsL2 = false, buttonsR2 = false, buttonsL1 = false, buttonsR1 = false;
boolean getPosOnce = false;
boolean reachedEndCrossed = false;
boolean autoAdjust = false, discloadFailed = false;
boolean autoMode = false;
boolean autoStopOnCenterPos = false;
boolean settingNextDisc = false;

void setPinmode() {
  pinMode(panMot_1, OUTPUT);
  pinMode(panMot_2, OUTPUT);
  pinMode(panPwm, OUTPUT);
  pinMode(discPullMot_1, OUTPUT);
  pinMode(discPullMot_2, OUTPUT);
  pinMode(discPullPwm, OUTPUT);
  pinMode(discMot_1, OUTPUT);
  pinMode(discMot_2, OUTPUT);
  pinMode(discPwm, OUTPUT);
  pinMode(startKillSwitch, INPUT_PULLUP);
  pinMode(endKillSwitch, INPUT_PULLUP);
  pinMode(led_1, OUTPUT);
  pinMode(led_2, OUTPUT);
  pinMode(led_3, OUTPUT);
  pinMode(discloadLimitSwt, INPUT_PULLUP);
  pinMode(anglePot, INPUT);

  pinMode(A14, OUTPUT);
  digitalWrite(A14, HIGH);   // give 5v to pot
}

int panPwmVal = 100;
int panCount = 95, prevPos = -1, i = -1;
static unsigned long last_interrupt_time = 0;
unsigned long interrupt_time = millis();
int debounce_time = 1;
boolean motionStat = false, go = false;
int stopCount = panCount;

void panCountInc() {
  interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > debounce_time) {
    if (motionStat) {
      digitalWrite(panMot_1, HIGH);
      digitalWrite(panMot_2, HIGH);
      analogWrite(panPwm, 255);
      detachInterrupt(digitalPinToInterrupt(panInterruptPin));
      prevPos = panCount;
      if (go) {
        panCount++;
      } else {
        panCount--;
      }
      motionStat = false;
      sendDataToLcd();
    }
    if (DEBUG) {
      Serial.print(" prevCount : ");
      Serial.print(prevPos);
      Serial.print("\tCount : ");
      Serial.print(panCount);
      Serial.print("\tstopCount : ");
      Serial.println(stopCount);
    }
    /*
      if (panCount > 95) {
      panPwmVal = 90;
      } else if (panCount < 95) {
      panPwmVal = 90;
      }
    */
    analogWrite(panPwm, panPwmVal);
    if (panCount == stopCount) {
      digitalWrite(panMot_1, HIGH);
      digitalWrite(panMot_2, HIGH);
      analogWrite(panPwm, 255);
      if (DEBUG) {
        Serial.print("Stopped at Count : ");
        Serial.println(stopCount);
      }
      autoAdjust = false;
    }

    if (panCount >= 190 || panCount <= 0) { // set min and max values
      digitalWrite(panMot_1, HIGH);
      digitalWrite(panMot_2, HIGH);
      analogWrite(panPwm, 255);
      if (DEBUG) {
        Serial.println("Limit reached");
      }
    }

    if (panCount > prevPos || panCount < prevPos) {}
    last_interrupt_time = interrupt_time;
    attachInterrupt(digitalPinToInterrupt(panInterruptPin), panCountInc, CHANGE);                       // try removing this
  }
  sendDataToLcd();
}

void sendDataToLcd() {                                       // send RPM, Angle, Pan, Velocity, Upper bot or Chasis, Sensor Values
  str = ":";
  if (autoAdjust) {
    str = str + autoAdjust + ',' + command + ',' + autoAdjustCount + ',' + autoValues[autoAdjustCount][0] + ',' + autoValues[autoAdjustCount][1] + ',' + autoValues[autoAdjustCount][2] +  ',' + throwingMotor + ',' + angleVal + ',' + panCount + ',' + loadingMotorState + ',' + lineON + ',' + ';';
  } else {
    str = str + autoAdjust + ',' + command + ',' + autoAdjustCount + ',' + (int)desired_rpm + ',' + throwingMotor + ',' + angleVal + ',' + panCount + ',' + loadingMotorState + ',' + lineON + ',' + ';';
  }
  BTserial.print(str);
}

void autoAdjustProc(int autoAdjustCount) {
  if (autoAdjustCount != 0) {
    Serial1.print(autoValues[autoAdjustCount][0]);                // send the RPM
    desired_rpm = autoValues[autoAdjustCount][0];
    if (DEBUG) {
      Serial.print("Desired RPM set to ");
      Serial.println(desired_rpm);
    }
    if (DEBUG) {
      Serial.println("RPM sent");
    }

    /*
      angleVal = analogRead(anglePot);
      //angleVal = map(angleVal, 300, 750, 0, 90);
      sendDataToLcd();
      if (DEBUG) {
        Serial.print("Angle Val : ");
        Serial.println(angleVal);
      }
      if (autoValues[autoAdjustCount][1] > angleVal) {              // adjust to the angle
        digitalWrite(angleMot_1, HIGH);
        digitalWrite(angleMot_2, LOW);
        analogWrite(anglePwm, 200);
        if (DEBUG) {
          Serial.println("Angle is greater move f");
        }
        sendDataToLcd();
      } else if (autoValues[autoAdjustCount][1] < angleVal) {
        digitalWrite(angleMot_1, LOW);
        digitalWrite(angleMot_2, HIGH);
        analogWrite(anglePwm, 200);
        if (DEBUG) {
          Serial.println("Angle is less move r");
        }
        sendDataToLcd();
      } else {
        digitalWrite(angleMot_1, HIGH);
        digitalWrite(angleMot_2, HIGH);
        analogWrite(anglePwm, 50);
        if (DEBUG) {
          Serial.println("Angle is Set");
        }
        sendDataToLcd();
      }
      /*
      stopCount = autoValues[autoAdjustCount][2];
      if (DEBUG) {
      Serial.println("Adjusting Pan");
      Serial.print("Stop Count : ");
      Serial.println(stopCount);
      }
      motionStat = false;                                // adjust to the given pan
      attachInterrupt(digitalPinToInterrupt(panInterruptPin), panCountInc, CHANGE);
      if (panCount < autoValues[autoAdjustCount][2]) {
      while (stopCount != panCount)
        autoPan(false);
      if (DEBUG) {
        Serial.println("Pan is less");
      }
      } else if (panCount > autoValues[autoAdjustCount][2]) {
      while (stopCount != panCount)
        autoPan(true);
      if (DEBUG) {
        Serial.println("Pan is greater");
      }
      go = false;
      } else {
      digitalWrite(panMot_1, HIGH);
      digitalWrite(panMot_2, HIGH);
      analogWrite(panPwm, 255);
      if (DEBUG) {
        Serial.println("Already on that Count");
      }
      }
      last_interrupt_time = millis();
      /*
      if (autoValues[autoAdjustCount][1] == angleVal &&/* panCount == autoValues[autoAdjustCount][2]) {
      if (DEBUG) {
      Serial.println("Everything is Set");
      }
      digitalWrite(angleMot_1, HIGH);
      digitalWrite(angleMot_2, HIGH);
      analogWrite(anglePwm, 50);
      digitalWrite(panMot_1, HIGH);
      digitalWrite(panMot_2, HIGH);
      analogWrite(panPwm, 255);
      autoAdjustCount = -1;
      upperBotControl = true;
      autoAdjust = false;
      }*/
    autoAdjustCount = 0;
    autoAdjust = false;
    sendDataToLcd();
  }
}

void setup() {
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

  Serial.println(F("\r\nPS2 Library Started"));

  mySerial.begin(9600);                                      // for communication with chasis
  BTserial.begin(9600);                                      // for communication with rpm sender

  Serial1.begin(9600);                        // for rpm transmission

  setPinmode();

  Serial1.print(desired_rpm);
  angleVal = analogRead(anglePot);
  Serial.println("All Set , RUN the bot");
  sendDataToLcd();
}

// Forward : F
// Reverse : B
// Left : L
// Right : R
// Stop : S
// Hard Stop : H
// LineFollow Forward : A
// LineFollow Backward : Z
// Position Left : O
// Position Right : P
// Automatic mode : Q
// Auto mode : D

void loop() {
  Usb.Task();

  if (loadingON) {
    if ((!digitalRead(endKillSwitch) || ((discloadStartTime + 2000) < millis())) && !reachedEnd) {
      digitalWrite(discMot_1, HIGH);
      digitalWrite(discMot_2, HIGH);
      analogWrite(discPwm, 50);
      if (DEBUG) {
        Serial.println("Reached end max");
      }
      discWaitTime = millis();
      reachedEnd = true;
      reachedEndCrossed = false;
      loadingMotorState = 2;
      sendDataToLcd();
    }
    if (millis() > (discWaitTime + 200) && reachedEnd && !reachedEndCrossed) {
      digitalWrite(discMot_1, LOW);
      digitalWrite(discMot_2, HIGH);
      analogWrite(discPwm, 50);
      if (DEBUG) {
        Serial.println("Going towards start");
      }
      discloadEndTime = millis();
      reachedEndCrossed = true;
      loadingMotorState = 3;
      sendDataToLcd();
    }
    if ((!digitalRead(startKillSwitch) || ((discloadEndTime + 2000) < millis())) && reachedEndCrossed) {
      digitalWrite(discMot_1, HIGH);
      digitalWrite(discMot_2, HIGH);
      analogWrite(discPwm, 50);
      if (DEBUG) {
        Serial.println("Reached Start max");
      }
      reachedEnd = false;
      reachedEndCrossed = false;
      loadingON = false;

      discloadingStartTime = millis();

      digitalWrite(discPullMot_1, HIGH);
      digitalWrite(discPullMot_2, LOW);
      analogWrite(discPullPwm, 100);
      if (DEBUG) {
        Serial.println("Pulling Disc to Switch");
      }
      loadingMotorState = 4;
      sendDataToLcd();
      settingNextDisc = true;
    }
  }
  if (settingNextDisc) {
    if (DEBUG) {
      Serial.println("Waiting for disc to reach Limit");
    }
    if (!digitalRead(discloadLimitSwt)) {
      // stop the rmcs motor
      // pause for a second
      if (DEBUG) {
        Serial.println("Disc Loaded on Position");
      }
      digitalWrite(discPullMot_1, HIGH);
      digitalWrite(discPullMot_2, HIGH);
      analogWrite(discPullPwm, 50);
      if (DEBUG) {
        Serial.println("Stopped Disc pull");
      }
      delay(300);
      digitalWrite(discPullMot_1, LOW);
      digitalWrite(discPullMot_2, HIGH);
      analogWrite(discPullPwm, 100);
      if (DEBUG) {
        Serial.println("Release Disc to make space");
      }
      delay(150);
      digitalWrite(discPullMot_1, LOW);
      digitalWrite(discPullMot_2, LOW);
      analogWrite(discPullPwm, 0);
      if (DEBUG) {
        Serial.println("Stopped Disc pull");
      }
      loadingMotorState = 5;
      sendDataToLcd();
      delay(200);
      loadingMotorState = 0;
      sendDataToLcd();
      digitalWrite(led_3, LOW);
      settingNextDisc = false;
    }
    if (discloadingStartTime + 2000 < millis()) { // break loop if disc not loaded in 4 seconds
      if (DEBUG) {
        Serial.println("Disc loading Time expired");
      }
      //discloadFailed = true;
      //loadingON = false;
      digitalWrite(discPullMot_1, LOW);
      digitalWrite(discPullMot_2, HIGH);
      analogWrite(discPullPwm, 100);
      if (DEBUG) {
        Serial.println("Release Disc to make space");
      }
      delay(300);
      digitalWrite(discPullMot_1, LOW);
      digitalWrite(discPullMot_2, LOW);
      analogWrite(discPullPwm, 0);
      if (DEBUG) {
        Serial.println("Stopped Disc pull");
      }
      loadingMotorState = 5;
      sendDataToLcd();
      delay(200);
      loadingMotorState = 0;
      sendDataToLcd();
      digitalWrite(led_3, LOW);
      settingNextDisc = false;
    }
  }

  if (angleButtonPressed || angleButtonPressedT) {
    angleVal = analogRead(anglePot);
    //angleVal = map(angleVal, 300, 750, 0, 90);
    if (DEBUG) {
      Serial.print("Angle Val : ");
      Serial.println(angleVal);
    }
    sendDataToLcd();
  }

  /*
        if (!digitalRead(angleResetKillSwt) && !getPosOnce) {
          getPosOnce = true;
          digitalWrite(angleMot_1, HIGH);
          digitalWrite(angleMot_2, HIGH);
          analogWrite(anglePwm, 50);
        }
        if (digitalRead(angleResetKillSwt) && getPosOnce) {
          getPosOnce = false;
        }
  */
}

#define Up x==127&&y>127
#define Down x==127&&y<127
#define Left x>127&&y==127
#define Right x<127&&y==127
#define nothing x==127&&y==127
#define Triangle but_id==1
#define Circle but_id==2
#define Cross but_id==3
#define Square but_id==4
#define Left_1 but_id==5
#define Right_1 but_id==6
#define Left_2 but_id==7
#define Right_2 but_id==8
#define Select but_id==9
#define Start but_id==10
#define Left_3 but_id==11
#define Right_3 but_id==12

void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt) {
  x = evt->X;
  y = evt->Y;

  if (Up && !buttonsUP) {
    if (throwingMotor) {
      command = 'P';
      mySerial.write("P");
      if (DEBUG) {
        Serial.println("SELF ADJUST ON LEFT SPOT");
      }
    } else {
      if (lineON) {
        command = 'A';
        mySerial.write("A");                                       // code that runs the line follower forward code
        if (DEBUG) {
          Serial.println("Line Following towards loading area");
        }
      } else {
        command = 'F';
        mySerial.write("F");                                       // go forward
        if (DEBUG) {
          Serial.println("GOING FORWARD");
        }
      }
    }
    buttonsUP = true;
    sendDataToLcd();
  }
  if (!Up && buttonsUP) {
    command = 'S';
    mySerial.write("S");
    buttonsUP = false;
    if (DEBUG) {
      Serial.println("UP BUTTON REALEASED STOPPED");
    }
    sendDataToLcd();
  }

  if (Down !buttonsDOWN) {
    if (throwingMotor) {
      command = 'O';
      mySerial.write("O");
      if (DEBUG) {
        Serial.println("SELF ADJUST ON RIGHT SPOT");
      }
    } else {
      if (lineON) {
        command = 'Z';
        mySerial.write("Z");
        if (DEBUG) {
          Serial.println("Line Following Reverse");
        }
      } else {
        command = 'B';
        mySerial.write("B");
        if (DEBUG) {
          Serial.println("GOING REVERSE");
        }
      }
    }
    buttonsDOWN = true;
    sendDataToLcd();
  }
  if (!Down && buttonsDOWN) {
    command = 'S';
    mySerial.write("S");
    buttonsDOWN = false;
    if (DEBUG) {
      Serial.println("DOWN BUTTON REALEASED STOPPED");
    }
    sendDataToLcd();
  }

  if (Left !buttonsLEFT) {
    if (lineON) {
      command = 'N';
      mySerial.write("N");
      if (DEBUG) {
        Serial.println("Position by count on Left");
      }
    } else {
      command = 'L';
      mySerial.write("L");
      if (DEBUG) {
        Serial.println("GOING LEFT TURN");
      }
    }
    buttonsLEFT = true;
    sendDataToLcd();
  }
  if (!Left && buttonsLEFT) {
    command = 'S';
    mySerial.write("S");
    buttonsUP = false;
    if (DEBUG) {
      Serial.println("LEFT BUTTON REALEASED STOPPED");
    }
    sendDataToLcd();
  }

  if (Right !buttonsRIGHT) {
    if (lineON) {
      command = 'V';
      mySerial.write("V");
      if (DEBUG) {
        Serial.println("Position by count on Right");
      }
    } else {
      command = 'R';
      mySerial.write("R");
      if (DEBUG) {
        Serial.println("GOING RIGHT TURN");
      }
    }
    buttonsRIGHT = true;
    sendDataToLcd();
  }
  if (!Right && buttonsRIGHT) {
    command = 'S';
    mySerial.write("S");
    buttonsRIGHT = false;
    if (DEBUG) {
      Serial.println("RIGHT BUTTON REALEASED STOPPED");
    }
    sendDataToLcd();
  }
}

void JoystickEvents::OnHatSwitch(uint8_t hat) {

}

void JoystickEvents::OnButtonDn(uint8_t but_id) {
  if (Select) {
    if (throwingMotor) {
      throwingMotor = false;
      Serial1.print("-1");
      Serial1.flush();
      if (DEBUG) {
        Serial.println("Throwing Motor is Stopped");
      }
      digitalWrite(led_2, LOW);
    } else {
      throwingMotor = true;
      Serial1.print("-2 ");
      Serial1.flush();
      Serial1.print(desired_rpm);
      if (DEBUG) {
        Serial.println("Throwing Motor is Start");
      }
      throwingStarted = millis();
      digitalWrite(led_2, HIGH);
    }
    if (DEBUG) {
      Serial.print(" Throwing Motor : ");
      Serial.println(throwingMotor);
    }
    sendDataToLcd();
  }

  if (Start) {
    //use rmcs to load next disc
    //discloadFailed = false;
    // reverse the operation
    digitalWrite(led_3, HIGH);
    loadingON = true;
    loadingMotorState = 1;
    sendDataToLcd();
    //if (!discloadFailed) {
    for (int i = 0; i <= 90; i++) {
      digitalWrite(discMot_1, HIGH);
      digitalWrite(discMot_2, LOW);
      analogWrite(discPwm, i);
      delayMicroseconds(1500);
    }
    //}
    discloadStartTime = millis();                     // for discload security feature
    if (DEBUG) {
      Serial.print("discloadStartTime : ");
      Serial.println(discloadStartTime);
      Serial.println("Disc Loading started");
    }
    sendDataToLcd();
  }

  if (Triangle) {
    if (autoAdjust) {
      autoAdjustCount = 1;
      autoAdjustProc(autoAdjustCount);
      if (DEBUG) {
        Serial.println("SPOT 1 - EXTREME LEFT");
      }
    } else {
      //if (!getPosOnce) {
      digitalWrite(angleMot_1, HIGH);
      digitalWrite(angleMot_2, LOW);
      analogWrite(anglePwm, 180);
      angleButtonPressedT = true;
      if (DEBUG) {
        Serial.print("Increasing Angle : ");
        Serial.println("180");
      }
      //}
    }
    sendDataToLcd();
  }

  if (Cross) {
    if (autoAdjust) {
      autoAdjustCount = 3;
      autoAdjustProc(autoAdjustCount);
      if (DEBUG) {
        Serial.println("SPOT 3 - ONE POINT SPOT");
      }
    } else {
      if (autoMode) {
        autoModeCount++;
        if (autoModeCount >= 4) {
          autoModeCount = 0;
          autoMode = false;
          if (DEBUG) {
            Serial.print("AutoMode : ");
            Serial.println(autoMode);
          }
          digitalWrite(led_1, LOW);
        }
        if (autoModeCount == 1) {
          command = 'G';
          mySerial.write("G");
        } else {
          command = 'S';
          mySerial.write("S");
        }
        buttonsRIGHT = false;
        if (DEBUG) {
          Serial.print("AutoMode OFF : ");
          Serial.println(autoModeCount);
        }
      } else {
        if (!throwingMotor) {
          command = 'S';
          mySerial.write("S");
          if (DEBUG) {
            Serial.println("Manually Stopped");
          }
        } else {
          //if (!getPosOnce) {
          digitalWrite(angleMot_1, LOW);
          digitalWrite(angleMot_2, HIGH);
          analogWrite(anglePwm, 180);
          angleButtonPressed = true;
          if (DEBUG) {
            Serial.print("Decreasing Angle : ");
            Serial.println("180");
          }
          //      } else {
          //        if (DEBUG) {
          //          Serial.println("Reached Kill Switch");
          //        }
        }
      }
    }
    sendDataToLcd();
  }

  if (Circle) {
    if (autoAdjust) {
      autoAdjustCount = 2;
      autoAdjustProc(autoAdjustCount);
      if (DEBUG) {
        Serial.println("SPOT 2 - LEFT 1 METER");
      }
    } else {
      motionStat = true;
      attachInterrupt(digitalPinToInterrupt(panInterruptPin), panCountInc, CHANGE);
      //        if (panCount <= 190 || panCount >= 0) {
      digitalWrite(panMot_1, HIGH);
      digitalWrite(panMot_2, LOW);
      analogWrite(panPwm, panPwmVal);
      //        }
      go = false;
      last_interrupt_time = millis();
      if (DEBUG) {
        Serial.println("Pan to Right");
      }
    }
  }

  if (Square) {
    if (autoAdjust) {
      autoAdjustCount = 4;
      autoAdjustProc(autoAdjustCount);
      if (DEBUG) {
        Serial.println("SPOT 4 - MIDDLE LARGE");
      }
    } else {
      motionStat = true;
      attachInterrupt(digitalPinToInterrupt(panInterruptPin), panCountInc, CHANGE);
      //        if (panCount <= 190 || panCount >= 0) {
      digitalWrite(panMot_1, LOW);
      digitalWrite(panMot_2, HIGH);
      analogWrite(panPwm, panPwmVal);
      //        }
      go = true;
      last_interrupt_time = millis();
      if (DEBUG) {
        Serial.println("Pan to Left");
      }
    }
  }

  if (Left_1) {
    if (autoAdjust) {
      autoAdjustCount = 5;
      autoAdjustProc(autoAdjustCount);
      if (DEBUG) {
        Serial.println("SPOT 5 - AWESOME 5 POINTS");
      }
    } else {
      if (!throwingMotor) {
        autoMode = true;
        command = 'D';
        mySerial.write("D");                                                       // call to reverse line follower code
        if (DEBUG) {
          Serial.println("Automatic going towards Loading Area");
        }
        digitalWrite(led_1, HIGH);
      } else {
        desired_rpm = desired_rpm - rpmIncrementVal;                      // check for invalid rpm values later
        Serial1.print(desired_rpm);
        if (DEBUG) {
          Serial.print(" RPM set to : ");
          Serial.println(desired_rpm);
        }
      }
    }
    sendDataToLcd();
  }

  if (Right_2) {
    if (autoAdjust) {
      autoAdjustCount = 6;
      autoAdjustProc(autoAdjustCount);
      if (DEBUG) {
        Serial.println("SPOT 6 - RIGHT 1 METER");
      }
    } else {
      if (!throwingMotor) {
        command = 'Q';
        mySerial.write("Q");                                                       // call to reverse line follower code
        if (DEBUG) {
          Serial.println("Going Automatic checking Junction");
        }
        autoStopOnCenterPos = true;
      } else {
        desired_rpm = desired_rpm + rpmIncrementVal;                      // check for invalid rpm values later
        Serial1.print(desired_rpm);
        if (DEBUG) {
          Serial.print(" RPM set to : ");
          Serial.println(desired_rpm);
        }
      }
    }
    sendDataToLcd();
  }

  if (Left_2) {
    if (autoAdjust) {
      autoAdjustCount = 7;
      autoAdjustProc(autoAdjustCount);
      if (DEBUG) {
        Serial.println("SPOT 7 - EXTREME RIGHT");
      }
    } else {
      digitalWrite(discPullMot_1, LOW);
      digitalWrite(discPullMot_2, HIGH);
      analogWrite(discPullPwm, 90);
      if (DEBUG) {
        Serial.println("Release Disc's");
      }
      buttonsL2 = true;
    }
  }

  if (Right_2) {
    //if (!autoStopOnCenterPos) {
    digitalWrite(discPullMot_1, HIGH);
    digitalWrite(discPullMot_2, LOW);
    analogWrite(discPullPwm, 90);
    if (DEBUG) {
      Serial.println("Press Disc's");
    }
    buttonsR2 = true;
    /* } else {
       command = 'S';
       mySerial.write("S");
       if (DEBUG) {
         Serial.println("STOPPED BY COUNT");
       }
       autoStopOnCenterPos = false;
      }
    */
    sendDataToLcd();
  }

  if (Left_3) {
    if (lineON == false) {
      lineON = true;
    } else if (lineON == true) {
      lineON = false;
    }
    if (DEBUG) {
      Serial.print("LINE FOLLOWER :"); Serial.println(lineON);
    }
    sendDataToLcd();
    loadingON = false;
    motionStat = false;
    digitalWrite(discMot_1, HIGH);
    digitalWrite(discMot_2, LOW);
    analogWrite(discPwm, 0);
    digitalWrite(panMot_1, HIGH);
    digitalWrite(panMot_2, HIGH);
    analogWrite(panPwm, 255);
    detachInterrupt(digitalPinToInterrupt(panInterruptPin));
    if (DEBUG) {
      Serial.println("Pan and Disc Motor Stopped, RMCS position reset");
    }
    digitalWrite(angleMot_1, HIGH);
    digitalWrite(angleMot_2, HIGH);
    analogWrite(anglePwm, 50);

    digitalWrite(discPullMot_1, LOW);
    digitalWrite(discPullMot_2, LOW);
    analogWrite(discPullPwm, 0);
    panCount = 95;
    sendDataToLcd();
  }

  if (Right_3) {
    if (autoAdjust) {
      autoAdjust = false;                                            // start auto adjust
    }
    else {
      autoAdjust = true;
    }
    if (DEBUG) {
      Serial.print("autoAdjust : ");
      Serial.println(autoAdjust);
    }
    sendDataToLcd();
  }
}


void JoystickEvents::OnButtonUp(uint8_t but_id) {
  if (Cross) {
    digitalWrite(angleMot_1, HIGH);
    digitalWrite(angleMot_2, HIGH);
    analogWrite(anglePwm, 50);
    angleButtonPressed = false;
    if (DEBUG) {
      Serial.println("Stopped Angle Adjust motor");
    }
    sendDataToLcd();
  } else if (Triangle) {
    digitalWrite(angleMot_1, HIGH);
    digitalWrite(angleMot_2, HIGH);
    analogWrite(anglePwm, 50);
    // stop the angle motor
    angleButtonPressedT = false;
    if (DEBUG) {
      Serial.println("Stopped Angle Adjust motor");
    }
    sendDataToLcd();
  }

  if (Left_2) {
    digitalWrite(discPullMot_1, HIGH);
    digitalWrite(discPullMot_2, HIGH);
    analogWrite(discPullPwm, 50);
    if (DEBUG) {
      Serial.println("Stopped Disc insert motor");
    }
    buttonsL2 = false;
  }

  if (Right_2) {
    digitalWrite(discPullMot_1, HIGH);
    digitalWrite(discPullMot_2, HIGH);
    analogWrite(discPullPwm, 50);
    if (DEBUG) {
      Serial.println("Stopped Disc insert motor");
    }
    buttonsR2 = false;
  }
}
