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

// SERIAL COMMUNICATION
// Serial 1 : send receive rpm
// Serial 2 : rmcs for angle
// Serial 3 : rmcs for disc load
// mySerial : communicate with chasis
// BTserial : bluetooth serial module

#define DEBUG 1                         // Serial print on or off in loop()

#include <SoftwareSerial.h>
SoftwareSerial mySerial(7, 35); // RX, TX                        // for communication
SoftwareSerial BTserial(28, 26); // RX | TX                       // for bluetooth

#include <PS3BT.h>
#include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd, 0x00, 0x1b, 0x10, 0x00, 0x2a, 0xec);                            // set address of bluetooth dongle

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
int autoAdjustCount = 0, autoModeCount = 0;

unsigned int rmcsAngleValues[7] = {0, 0, 0, 0, 0, 0, 0};                        // rmcs position values for each spot
int autoValues[8][3] = { {0, 0, 0},
  {3340, 75, 90},  // Spot 1
  {3480, 70, 95},  // Spot 2
  {2500, 70, 100},  // Spot 3
  {3520, 70, 96},  // Spot 4
  {4300, 70, 92},  // Spot 5
  {3480, 70, 97},  // Spot 6
  {3340, 70, 95}   // Spot 7
};
//RPM , ANGLE, PAN

//boolean values
boolean buttonsUP = false, buttonsDOWN = false;
boolean buttonsLEFT = false, buttonsRIGHT = false;                              // for Joystick pressed loop operation
boolean lineFollowFront = false, lineFollowReverse = false;                     // run line follow code , forward or reverse
boolean lineON = true;                                         // decide wheather line follower on or off and invert mode
boolean automaticMode = false, stopControl = false;                             // automatic mode and stopControl for bluetooth disconnection control
boolean which = false;
boolean angleButtonPressed = false, angleButtonPressedT = false;
boolean throwingMotor = false;
boolean loadingON = false;
boolean reachedEnd = false;
boolean buttonsL2 = false, buttonsR2 = false, buttonsL1 = false, buttonsR1 = false;
boolean getRotationVal = false;
boolean getPosOnce = false;
boolean incPan = false;
boolean reachedEndCrossed = false;
boolean autoAdjust = false, discloadFailed = false;
boolean autoMode = false;
boolean autoStopOnCenterPos = false;
boolean settingNextDisc = false;

#define panMot_1 13//29
#define panMot_2 11//27
#define panPwm 12//5

#define discPullMot_1 33
#define discPullMot_2 31
#define discPullPwm 6

#define panInterruptPin 20

#define discMot_1 23
#define discMot_2 25
#define discPwm 4

#define startKillSwitch 39  // grey white
#define endKillSwitch 41   // purple blue

#define angleRheostatPin A0

//#define discloadRmcsReset 49

#define discloadLimitSwt 43 // green yellow

#define led_1 32
#define led_2 34
#define led_3 36
//#define led_4 22

void setPinmode() {
  //  pinMode(loadingMot_1, OUTPUT);
  //  pinMode(loadingMot_2, OUTPUT);
  //  pinMode(loadingPWM, OUTPUT);*
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
  // pinMode(led_4, OUTPUT);
  //  pinMode(angleResetKillSwt, INPUT_PULLUP);
  //  pinMode(discloadRmcsReset, INPUT_PULLUP);
  pinMode(discloadLimitSwt, INPUT_PULLUP);
  pinMode(angleRheostatPin, INPUT);

  pinMode(A1, OUTPUT);
  digitalWrite(A1, HIGH);   // give 5v to rheostat wire
}

void getRmcsPos() {
  // get rmcs pos here
}

int panPwmVal = 100;
int panCount = 95, prevPos = -1, i = -1;
static unsigned long last_interrupt_time = 0, startTime = 0;
unsigned long interrupt_time = millis(), start, End;
int debounce_time = 1, setpoint = 0;
boolean motionStat = false, panOn = false, go = false;;
int stopCount;

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

void Display(boolean which) {
  if (which) {
    while (Serial2.available()) {
      if (DEBUG) {
        Serial.print((char)Serial2.read());
      }
    }
    if (DEBUG)
      Serial.println("");
  }
  if (DEBUG)
    Serial.println("");
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
      angleVal = analogRead(angleRheostatPin);
      angleVal = map(angleVal, 300, 750, 0, 90);
      sendDataToLcd();
      if (DEBUG) {
        Serial.print("Angle Val : ");
        Serial.println(angleVal);
      }
      if (autoValues[autoAdjustCount][1] > angleVal) {              // adjust to the angle
        Serial2.println(rmcsAngleSpeed);
        delay(200);
        if (DEBUG) {
          Serial.println("Angle is greater move f");
        }
        sendDataToLcd();
      } else if (autoValues[autoAdjustCount][1] < angleVal) {
        Serial2.println(rmcsAngleSpeedN);
        delay(200);
        if (DEBUG) {
          Serial.println("Angle is less move r");
        }
        sendDataToLcd();
      } else {
        Serial2.println("S0");
        delay(200);
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
      Serial2.println("S0");
      delay(200);
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
  mySerial.begin(9600);                                      // for communication with chasis
  BTserial.begin(9600);                                      // for communication with rpm sender

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for Serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC Serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.println(F("\r\nPS3 Bluetooth Library Started"));

  Serial1.begin(9600);                        // for rpm transmission
  Serial2.begin(9600);                        // rmcs for angle control

  setPinmode();

  Serial2.println("Y");
  delay(200);
  Display(true);
  Serial2.println("D0");
  delay(200);
  Display(true);
  Serial2.println("S0");
  delay(200);
  Display(true);
  Serial1.print(desired_rpm);
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

  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    stopControl = true;                                                           // see if bluetooth disconnected

    if (PS3.getButtonClick(UP)) {
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
    if (!PS3.getButtonPress(UP) && buttonsUP) {
      command = 'S';
      mySerial.write("S");
      buttonsUP = false;
      if (DEBUG) {
        Serial.println("UP REALEASED STOPPED");
      }
      sendDataToLcd();
    }

    if (PS3.getButtonClick(DOWN)) {
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
    if (!PS3.getButtonPress(DOWN) && buttonsDOWN) {
      command = 'S';
      mySerial.write("S");
      buttonsDOWN = false;
      if (DEBUG) {
        Serial.println("DOWN RELEASED STOPPED");
      }
      sendDataToLcd();
    }

    if (PS3.getButtonClick(LEFT)) {                                        // left turn
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
    if (!PS3.getButtonPress(LEFT) && buttonsLEFT) {
      command = 'S';
      mySerial.write("S");
      buttonsLEFT = false;
      if (DEBUG) {
        Serial.println("LEFT RELEASED STOPPED");
      }
      sendDataToLcd();
    }

    if (PS3.getButtonClick(RIGHT)) {                                       // right turn
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
    if (!PS3.getButtonPress(RIGHT) && buttonsRIGHT) {
      command = 'S';
      mySerial.write("S");
      buttonsRIGHT = false;
      if (DEBUG) {
        Serial.println("RIGHT RELEASED STOPPED");
      }
      sendDataToLcd();
    }

    if (PS3.getButtonClick(SELECT)) {
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

    if (PS3.getButtonClick(START)) {                                       // load the discs
      //use rmcs to load next disc
      //discloadFailed = false;
      // reverse the operation
      digitalWrite(led_3, HIGH);
      loadingON = true;
      loadingMotorState = 1;
      sendDataToLcd();
      //if (!discloadFailed) {
      for (int i = 0; i <= 255; i++) {
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
    if (loadingON) {
      if ((!digitalRead(endKillSwitch) || ((discloadStartTime + 2000) < millis())) && !reachedEnd) {
        digitalWrite(discMot_1, HIGH);
        digitalWrite(discMot_2, HIGH);
        analogWrite(discPwm, 255);
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
        analogWrite(discPwm, 255);
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
        analogWrite(discPwm, 255);
        if (DEBUG) {
          Serial.println("Reached Start max");
        }
        reachedEnd = false;
        reachedEndCrossed = false;
        loadingON = false;

        discloadingStartTime = millis();

        digitalWrite(discPullMot_1, HIGH);
        digitalWrite(discPullMot_2, LOW);
        analogWrite(discPullPwm, 85);
        if (DEBUG) {
          Serial.println("Pulling Disc to Switch");
        }

        Display(false);
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
        analogWrite(discPullPwm, 55);
        if (DEBUG) {
          Serial.println("Release Disc to make space");
        }
        delay(135);
        digitalWrite(discPullMot_1, LOW);
        digitalWrite(discPullMot_2, LOW);
        analogWrite(discPullPwm, 0);
        if (DEBUG) {
          Serial.println("Stopped Disc pull");
        }
        loadingMotorState = 5;
        sendDataToLcd();
        delay(500);
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
        analogWrite(discPullPwm, 70);
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
        delay(500);
        loadingMotorState = 0;
        sendDataToLcd();
        digitalWrite(led_3, LOW);
        settingNextDisc = false;
      }
    }

    if (PS3.getButtonClick(TRIANGLE)) {                                         // automatic set to center
      if (autoAdjust) {
        autoAdjustCount = 1;
        autoAdjustProc(autoAdjustCount);
        if (DEBUG) {
          Serial.println("SPOT 1 - EXTREME LEFT");
        }
      } else {
        //if (!getPosOnce) {
        Serial2.println(rmcsAngleSpeed);
        delay(200);
        angleButtonPressedT = true;
        if (DEBUG) {
          Serial.print("Increasing Angle : ");
          Serial.println(rmcsAngleSpeed);
          Display(true);
        }
        //}
      }
      sendDataToLcd();
    }

    if (PS3.getButtonClick(CROSS)) {
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
            Serial2.println(rmcsAngleSpeedN);
            delay(200);
            angleButtonPressed = true;
            if (DEBUG) {
              Serial.print("Decreasing Angle : ");
              Serial.println(rmcsAngleSpeedN);
              Display(true);
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

    if (!PS3.getButtonPress(CROSS) && angleButtonPressed) {
      Serial2.println("S0");                                                // stop the angle motor
      delay(200);
      angleButtonPressed = false;
      if (DEBUG) {
        Display(true);
        Serial.println("Stopped Angle Adjust motor");
      }
      sendDataToLcd();
    } else if (!PS3.getButtonPress(TRIANGLE) && angleButtonPressedT) {
      Serial2.println("S0");
      delay(200);
      // stop the angle motor
      angleButtonPressedT = false;
      if (DEBUG) {
        Display(true);
        Serial.println("Stopped Angle Adjust motor");
      }
      sendDataToLcd();
    }

    if (PS3.getButtonClick(CIRCLE)) {                                       // pan motor to left
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
    if (PS3.getButtonClick(SQUARE)) {                                       // pan motor to right
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

    if (PS3.getButtonClick(L1)) {
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

    if (PS3.getButtonClick(R1)) {
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

    if (PS3.getButtonClick(L2)) {
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
    if (!PS3.getButtonPress(L2) && buttonsL2) {
      digitalWrite(discPullMot_1, HIGH);
      digitalWrite(discPullMot_2, HIGH);
      analogWrite(discPullPwm, 50);
      if (DEBUG) {
        Serial.println("Stopped Disc insert motor");
      }
      buttonsL2 = false;
    }

    if (PS3.getButtonClick(R2)) {
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
    if (!PS3.getButtonPress(R2) && buttonsR2) {
      digitalWrite(discPullMot_1, HIGH);
      digitalWrite(discPullMot_2, HIGH);
      analogWrite(discPullPwm, 50);
      if (DEBUG) {
        Serial.println("Stopped Disc insert motor");
      }
      buttonsR2 = false;
    }
    if (angleButtonPressed || angleButtonPressedT) {
      angleVal = analogRead(angleRheostatPin);
      angleVal = map(angleVal, 300, 750, 0, 90);
      if (DEBUG) {
        Serial.print("Angle Val : ");
        Serial.println(angleVal);
      }
      sendDataToLcd();
    }

    /*
          if (!digitalRead(angleResetKillSwt) && !getPosOnce) {
            getPosOnce = true;
            Serial2.println("P0");
            delay(200);
            Display(true);
            Serial2.println("S0");
            delay(200);
            Display(true);
          }
          if (digitalRead(angleResetKillSwt) && getPosOnce) {
            getPosOnce = false;
          }
    */
    if (PS3.getButtonClick(L3)) {                                    // switch controls from chasis to upper bot
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
      Serial2.println("S0");
      delay(200);
      Display(true);
      digitalWrite(discPullMot_1, LOW);
      digitalWrite(discPullMot_2, LOW);
      analogWrite(discPullPwm, 0);
      panCount = 95;
      sendDataToLcd();
    }

    if (PS3.getButtonClick(R3)) {
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
  } else {
    if (stopControl == true) {
      mySerial.write("S");
    }
  }
}
