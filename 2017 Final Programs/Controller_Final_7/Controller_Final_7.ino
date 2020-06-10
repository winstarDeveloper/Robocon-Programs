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
// removed rmcs
// L1 and R1 buttons modified

// SERIAL COMMUNICATION
// Serial 1 : send receive rpm
// Serial 2 : free
// Serial 3 : free
// mySerial : communicate with chasis
// BTserial : bluetooth serial module

#define DEBUG 1                         // Serial print on or off in loop()

#include <SoftwareSerial.h>
SoftwareSerial mySerial(7, 35); // RX, TX                        // for communication
SoftwareSerial BTserial(40, 38); // RX | TX                       // for bluetooth

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
double desired_rpm = 3520;
unsigned int rpmIncrementVal = 10;
String rmcsAngleSpeed = "S-100", rmcsAngleSpeedN = "S160"; // change to pwm values
static unsigned long discPushWaitTime = 1000, discWaitTime, discloadStartTime, discloadEndTime, throwingStarted, discloadingStartTime;
unsigned int angleVal = 0;
int command = -1;
String str = ":";
unsigned int loadingMotorState = 0, startPanVal = 0, endPanVal = 0;
int autoAdjustCount = 0, autoModeCount = 0;
int panPwmVal = 115;
int panCount = 0/*95*/, prevPos = -1, i = -1;

int autoValues[8][3] = { {0, 0, 0},
  {3340, 180, 90},  // Spot 1
  {3480, 170, 95},  // Spot 2
  {2500, 185, 100}, // Spot 3
  {3520, 200, 96},  // Spot 4 - 210 , 276
  {4300, 190, 92},  // Spot 5
  {3480, 190, 97},  // Spot 6
  {3340, 190, 95}   // Spot 7
};
//RPM , ANGLE, PAN

//boolean values
boolean buttonsUP = false, buttonsDOWN = false;
boolean buttonsLEFT = false, buttonsRIGHT = false;                              // for Joystick pressed loop operation
boolean buttonsCIRCLE = false, buttonsSQUARE = false;
boolean buttonsL2 = false, buttonsR2 = false, buttonsL1 = false, buttonsR1 = false;
boolean lineFollowFront = false, lineFollowReverse = false;                     // run line follow code , forward or reverse
boolean lineON = true;                                         // decide wheather line follower on or off and invert mode
boolean automaticMode = false, stopControl = false;                             // automatic mode and stopControl for bluetooth disconnection control
boolean which = false;
boolean angleButtonPressed = false, angleButtonPressedT = false;
boolean throwingMotor = false;
boolean loadingON = false;
boolean reachedEnd = false;
boolean getRotationVal = false;
boolean getPosOnce = false;
boolean incPan = false;
boolean reachedEndCrossed = false;
boolean autoAdjust = false, discloadFailed = false;
boolean autoMode = false;
boolean autoStopOnCenterPos = false;
boolean settingNextDisc = false;
boolean panAdjust = false;

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

#define startKillSwitch 47  // grey white
#define endKillSwitch 49   // purple blue

#define anglePot A15
#define panPot A11

//#define discloadRmcsReset 49

#define discloadLimitSwt 43 // green yellow

#define led_1 46
#define led_2 48
#define led_3 50
//#define led_4 22

void sendDataToLcd(unsigned int a) {                                       // send RPM, Angle, Pan, Velocity, Upper bot or Chasis, Sensor Values
  str = ":";
  if (a == 0) {
    if (autoAdjust) {
      str = str + autoAdjust + ',' + command + ',' + autoAdjustCount + ',' + autoValues[autoAdjustCount][0] + ',' + autoValues[autoAdjustCount][1] + ',' + autoValues[autoAdjustCount][2] +  ',' + throwingMotor + ',' + angleVal + ',' + panCount + ',' + loadingMotorState + ',' + lineON + ',' + ';';
    } else {
      str = str + autoAdjust + ',' + command + ',' + autoAdjustCount + ',' + (int)desired_rpm + ',' + throwingMotor + ',' + angleVal + ',' + panCount + ',' + loadingMotorState + ',' + lineON + ',' + ';';
    }
  } else if (a == 1) {
    str = str + 2 + ',' + angleVal + ',' + ';';
  } else if (a == 2) {
    str = str + 3 + ',' + panCount + ',' + ';';
  }
  BTserial.print(str);
}

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
  pinMode(angleMot_1, OUTPUT);
  pinMode(angleMot_2, OUTPUT);
  pinMode(anglePwm, OUTPUT);
  pinMode(startKillSwitch, INPUT_PULLUP);
  pinMode(endKillSwitch, INPUT_PULLUP);
  pinMode(led_1, OUTPUT);
  pinMode(led_2, OUTPUT);
  pinMode(led_3, OUTPUT);
  // pinMode(led_4, OUTPUT);
  //  pinMode(angleResetKillSwt, INPUT_PULLUP);
  //  pinMode(discloadRmcsReset, INPUT_PULLUP);
  pinMode(discloadLimitSwt, INPUT_PULLUP);
  pinMode(anglePot, INPUT);
  pinMode(panPot, INPUT);

  pinMode(A14, OUTPUT);
  digitalWrite(A14, HIGH);   // give 5v to potentiometer angle
  pinMode(A10, OUTPUT);
  digitalWrite(A10, HIGH);   // give 5v to potentiometer pan
}

static unsigned long last_interrupt_time = 0;
unsigned long interrupt_time = millis();
int debounce_time = 1;
boolean motionStat = false, go = false;;
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
      sendDataToLcd(0);
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
  sendDataToLcd(0);
}

void autoAdjustProc(int autoAdjustCount) {
  if (autoAdjustCount != 0) {
    desired_rpm = autoValues[autoAdjustCount][0];
    if (desired_rpm != 13 && desired_rpm != 17)
      Serial1.print(desired_rpm);                // send the RPM
    if (DEBUG) {
      Serial.print("Desired RPM set to ");
      Serial.println(desired_rpm);
    }
    if (DEBUG) {
      Serial.println("RPM sent");
    }
    autoAdjustCount = 0;
    autoAdjust = false;
    sendDataToLcd(0);

    /*
      angleVal = analogRead(anglePot);
      //angleVal = map(angleVal, 300, 750, 0, 90);
      sendDataToLcd(1);
      if (DEBUG) {
        Serial.print("Angle Val : ");
        Serial.println(angleVal);
      }
      if (autoValues[autoAdjustCount][1] > angleVal) {              // adjust to the angle
        digitalWrite(angleMot_1, HIGH);
        digitalWrite(angleMot_2, LOW);
        analogWrite(anglePwm, 150);
        if (DEBUG) {
          Serial.println("Angle is greater move f");
        }
        sendDataToLcd(0);
      } else if (autoValues[autoAdjustCount][1] < angleVal) {
        digitalWrite(angleMot_1, LOW);
        digitalWrite(angleMot_2, HIGH);
        analogWrite(anglePwm, 150);
        if (DEBUG) {
          Serial.println("Angle is less move r");
        }
        sendDataToLcd(0);
      } else {
        digitalWrite(angleMot_1, HIGH);
        digitalWrite(angleMot_2, HIGH);
        analogWrite(anglePwm, 50);
        if (DEBUG) {
          Serial.println("Angle is Set");
        }
        //autoAdjust = false;
        sendDataToLcd(0);
      }

      stopCount = autoValues[autoAdjustCount][2];
      panCount = analogRead(panPot);
      sendDataToLcd(2);
      if (DEBUG) {
        Serial.print("Adjusting Pan");
        Serial.print("\tPan Count : ");
        Serial.print(panCount);
        Serial.print("\tStop Count : ");
        Serial.println(stopCount);
      }
      motionStat = false;                                // adjust to the given pan
      //attachInterrupt(digitalPinToInterrupt(panInterruptPin), panCountInc, CHANGE);
      if (panCount < autoValues[autoAdjustCount][2]) {
        digitalWrite(panMot_1, LOW);
        digitalWrite(panMot_2, HIGH);
        analogWrite(panPwm, panPwmVal);
        if (DEBUG) {
          Serial.println("Pan is less");
        }
      } else if (panCount > autoValues[autoAdjustCount][2]) {
        digitalWrite(panMot_1, HIGH);
        digitalWrite(panMot_2, LOW);
        analogWrite(panPwm, panPwmVal);
        if (DEBUG) {
          Serial.println("Pan is greater");
        }
      } else {
        digitalWrite(panMot_1, HIGH);
        digitalWrite(panMot_2, HIGH);
        analogWrite(panPwm, 255);
        if (DEBUG) {
          Serial.println("Already on that Pan Count");
        }
      }
      //last_interrupt_time = millis();

      angleVal = analogRead(anglePot);
      sendDataTolcd(1);
      if (autoValues[autoAdjustCount][1] == angleVal && panCount == autoValues[autoAdjustCount][2]) {
        if (DEBUG) {
          Serial.println("Everything is Set");
        }
        digitalWrite(angleMot_1, HIGH);
        digitalWrite(angleMot_2, HIGH);
        analogWrite(anglePwm, 50);
        digitalWrite(panMot_1, HIGH);
        digitalWrite(panMot_2, HIGH);
        analogWrite(panPwm, 255);
        autoAdjustCount = 0;
        autoAdjust = false;
        sendDataToLcd(1);
      }
    */
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

  setPinmode();
  if (desired_rpm != 13 && desired_rpm != 17)
    Serial1.print(desired_rpm);
  angleVal = analogRead(anglePot);
  panCount = analogRead(panPot);
  Serial.println("All Set , RUN the bot");
  sendDataToLcd(0);
}

// Forward : F
// Reverse : B
// Left : L
// Right : R
// SlowRight : T
// SlowLeft : Y
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

    if (autoAdjust) {
      autoAdjustProc(autoAdjustCount);
    }

    if (panAdjust) {
      endPanVal = analogRead(panPot);
      if (DEBUG) {
        Serial.print("startPanVal : ");
        Serial.print(startPanVal);
        Serial.print("\tendPanVal : ");
        Serial.println(endPanVal);
      }
      if (!go) {
        if (endPanVal > startPanVal + 5) {
          if (DEBUG) {
            Serial.println("Pan Motor Stopped");
          }
          digitalWrite(panMot_1, HIGH);
          digitalWrite(panMot_2, HIGH);
          analogWrite(panPwm, 255);
          panAdjust = false;
        }
      } else {
        if (endPanVal < startPanVal - 5) {
          if (DEBUG) {
            Serial.println("Pan Motor Stopped");
          }
          digitalWrite(panMot_1, HIGH);
          digitalWrite(panMot_2, HIGH);
          analogWrite(panPwm, 255);
          panAdjust = false;
        }
      }
      panCount = analogRead(panPot);
      if (DEBUG) {
        Serial.print("Pan Count: ");
        Serial.println(panCount);
      }
      sendDataToLcd(2);
    }

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
      sendDataToLcd(0);
    }
    if (!PS3.getButtonPress(UP) && buttonsUP) {
      command = 'S';
      mySerial.write("S");
      buttonsUP = false;
      if (DEBUG) {
        Serial.println("UP REALEASED STOPPED");
      }
      sendDataToLcd(0);
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
      sendDataToLcd(0);
    }
    if (!PS3.getButtonPress(DOWN) && buttonsDOWN) {
      command = 'S';
      mySerial.write("S");
      buttonsDOWN = false;
      if (DEBUG) {
        Serial.println("DOWN RELEASED STOPPED");
      }
      sendDataToLcd(0);
    }

    if (PS3.getButtonClick(LEFT)) {                                        // left turn
      if (lineON) {
        command = 'V';
        mySerial.write("V");
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
      sendDataToLcd(0);
    }
    if (!PS3.getButtonPress(LEFT) && buttonsLEFT) {
      command = 'S';
      mySerial.write("S");
      buttonsLEFT = false;
      if (DEBUG) {
        Serial.println("LEFT RELEASED STOPPED");
      }
      sendDataToLcd(0);
    }

    if (PS3.getButtonClick(RIGHT)) {                                       // right turn
      if (lineON) {
        command = 'N';
        mySerial.write("N");
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
      sendDataToLcd(0);
    }
    if (!PS3.getButtonPress(RIGHT) && buttonsRIGHT) {
      command = 'S';
      mySerial.write("S");
      buttonsRIGHT = false;
      if (DEBUG) {
        Serial.println("RIGHT RELEASED STOPPED");
      }
      sendDataToLcd(0);
    }

    if (PS3.getButtonClick(SELECT)) {
      if (throwingMotor) {
        throwingMotor = false;
        Serial1.print("13");
        Serial1.flush();
        if (DEBUG) {
          Serial.println("Throwing Motor is Stopped");
        }
        digitalWrite(led_2, LOW);
      } else {
        throwingMotor = true;
        Serial1.print("17 ");
        //Serial1.flush();
        if (desired_rpm != 13 && desired_rpm != 17)
          Serial1.print(desired_rpm);
        Serial1.flush();
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
      sendDataToLcd(0);
    }

    if (PS3.getButtonClick(START)) {                                       // load the discs
      //use rmcs to load next disc
      //discloadFailed = false;
      // reverse the operation
      digitalWrite(led_3, HIGH);
      loadingON = true;
      loadingMotorState = 1;
      sendDataToLcd(0);
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
      sendDataToLcd(0);
    }
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
        sendDataToLcd(0);
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
        sendDataToLcd(0);
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
        sendDataToLcd(0);
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
        delay(250);
        digitalWrite(discPullMot_1, LOW);
        digitalWrite(discPullMot_2, HIGH);
        analogWrite(discPullPwm, 80);
        if (DEBUG) {
          Serial.println("Release Disc to make space");
        }
        delay(100);
        digitalWrite(discPullMot_1, LOW);
        digitalWrite(discPullMot_2, LOW);
        analogWrite(discPullPwm, 0);
        if (DEBUG) {
          Serial.println("Stopped Disc pull");
        }
        loadingMotorState = 5;
        sendDataToLcd(0);
        delay(200);
        loadingMotorState = 0;
        sendDataToLcd(0);
        digitalWrite(led_3, LOW);
        settingNextDisc = false;
      }
      if (discloadingStartTime + 800 < millis()) { // break loop if disc not loaded in 4 seconds
        if (DEBUG) {
          Serial.println("Disc loading Time expired");
        }
        //discloadFailed = true;
        //loadingON = false;
        digitalWrite(discPullMot_1, LOW);
        digitalWrite(discPullMot_2, HIGH);
        analogWrite(discPullPwm, 80);
        if (DEBUG) {
          Serial.println("Release Disc to make space");
        }
        delay(120);
        digitalWrite(discPullMot_1, LOW);
        digitalWrite(discPullMot_2, LOW);
        analogWrite(discPullPwm, 0);
        if (DEBUG) {
          Serial.println("Stopped Disc pull");
        }
        loadingMotorState = 5;
        sendDataToLcd(0);
        delay(200);
        loadingMotorState = 0;
        sendDataToLcd(0);
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
        sendDataToLcd(0);
      } else {
        //if (!getPosOnce) {
        angleVal = analogRead(anglePot);
        if (angleVal < 1023) {
          digitalWrite(angleMot_1, HIGH);
          digitalWrite(angleMot_2, LOW);
          analogWrite(anglePwm, 180);
          angleButtonPressedT = true;
          if (DEBUG) {
            Serial.print("Increasing Angle : ");
            Serial.println("180");
          }
        }
        //}
      }
      //sendDataToLcd(0);
    }
    while (PS3.getButtonPress(TRIANGLE) && angleButtonPressedT) {
      angleVal = analogRead(anglePot);
      if (DEBUG) {
        Serial.print("Angle Val : ");
        Serial.println(angleVal);
      }
      sendDataToLcd(1);
      Usb.Task();
      if (angleVal >= 1023) {
        digitalWrite(angleMot_1, HIGH);
        digitalWrite(angleMot_2, HIGH);
        analogWrite(anglePwm, 50);
        angleButtonPressed = false;
        angleVal = analogRead(anglePot);
        if (DEBUG) {
          Serial.println("Stopped Angle Adjust motor - limit reached");
        }
        sendDataToLcd(1);
        break;
      }
      if (!PS3.getButtonPress(TRIANGLE)) {
        digitalWrite(angleMot_1, HIGH);
        digitalWrite(angleMot_2, HIGH);
        analogWrite(anglePwm, 50);
        // stop the angle motor
        angleVal = analogRead(anglePot);
        angleButtonPressedT = false;
        if (DEBUG) {
          Serial.println("Stopped Angle Adjust motor");
        }
        sendDataToLcd(1);
        break;
      }
      if (!PS3.PS3Connected) {
        digitalWrite(angleMot_1, HIGH);
        digitalWrite(angleMot_2, HIGH);
        analogWrite(anglePwm, 50);
        // stop the angle motor
        angleVal = analogRead(anglePot);
        angleButtonPressedT = false;
        if (DEBUG) {
          Serial.println("Stopped Angle Adjust motor - PS3 disconnected");
        }
        break;
      }
    }

    if (PS3.getButtonClick(CROSS)) {
      if (autoAdjust) {
        autoAdjustCount = 3;
        autoAdjustProc(autoAdjustCount);
        if (DEBUG) {
          Serial.println("SPOT 3 - ONE POINT SPOT");
        }
        sendDataToLcd(0);
      } else {
        if (autoMode) {
          autoModeCount++;
          if (autoModeCount >= 2) {
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
          sendDataToLcd(0);
        } else {
          //if (!getPosOnce) {
          angleVal = analogRead(anglePot);
          if (angleVal > 0) {
            digitalWrite(angleMot_1, LOW);
            digitalWrite(angleMot_2, HIGH);
            analogWrite(anglePwm, 180);
            angleButtonPressed = true;
            if (DEBUG) {
              Serial.print("Decreasing Angle : ");
              Serial.println("180");
            }
          }
          //      } else {
          //        if (DEBUG) {
          //          Serial.println("Reached Kill Switch");
          //        }
        }
      }
      //sendDataToLcd(0);
    }
    while (PS3.getButtonPress(CROSS) && angleButtonPressed) {
      angleVal = analogRead(anglePot);
      if (DEBUG) {
        Serial.print("Angle Val : ");
        Serial.println(angleVal);
      }
      sendDataToLcd(1);
      Usb.Task();
      if (angleVal <= 10) {
        digitalWrite(angleMot_1, HIGH);
        digitalWrite(angleMot_2, HIGH);
        analogWrite(anglePwm, 50);
        angleButtonPressed = false;
        angleVal = analogRead(anglePot);
        if (DEBUG) {
          Serial.println("Stopped Angle Adjust motor - limit reached");
        }
        sendDataToLcd(1);
        break;
      }
      if (!PS3.getButtonPress(CROSS)) {
        digitalWrite(angleMot_1, HIGH);
        digitalWrite(angleMot_2, HIGH);
        analogWrite(anglePwm, 50);
        angleButtonPressed = false;
        angleVal = analogRead(anglePot);
        if (DEBUG) {
          Serial.println("Stopped Angle Adjust motor");
        }
        sendDataToLcd(1);
        break;
      }
      if (!PS3.PS3Connected) {
        digitalWrite(angleMot_1, HIGH);
        digitalWrite(angleMot_2, HIGH);
        analogWrite(anglePwm, 50);
        // stop the angle motor
        angleVal = analogRead(anglePot);
        angleButtonPressedT = false;
        if (DEBUG) {
          Serial.println("Stopped Angle Adjust motor - PS3 disconnected");
        }
        break;
      }
    }
    /*
      //angleVal = analogRead(anglePot);
      //sendDataToLcd(1);
      if (angleButtonPressed || angleButtonPressedT) {
      angleVal = analogRead(anglePot);
      //angleVal = map(angleVal, 300, 750, 0, 90);
      /*
        if (angleVal >= 220 || angleVal <= 140) {
        digitalWrite(angleMot_1, HIGH);
        digitalWrite(angleMot_2, HIGH);
        analogWrite(anglePwm, 50);
        if (DEBUG) {
          Serial.println("Limit of Angle Motor reached");
        }
        }
      /
      if (DEBUG) {
        Serial.print("Angle Val : ");
        Serial.println(angleVal);
      }
      sendDataToLcd(1);
      }
    */
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

    if (PS3.getButtonClick(CIRCLE)) {                                       // pan motor to left
      if (autoAdjust) {
        autoAdjustCount = 2;
        autoAdjustProc(autoAdjustCount);
        if (DEBUG) {
          Serial.println("SPOT 2 - LEFT 1 METER");
        }
      } else {
        /*
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
        */
        if (lineON) {
          panAdjust = true;
          panCount = analogRead(panPot);
          sendDataToLcd(2);
          if (panCount <= 1012) {
            digitalWrite(panMot_1, LOW);
            digitalWrite(panMot_2, HIGH);
            analogWrite(panPwm, panPwmVal);
          }
          startPanVal = analogRead(panPot);
          go = true;
          if (DEBUG) {
            Serial.print("Pan to Right\t");
            Serial.print("Pan : ");
            Serial.println(startPanVal);
          }
        } else {
          command = 'T';
          mySerial.write("T");
          if (DEBUG) {
            Serial.println("Slow Right Motor");
          }
          buttonsCIRCLE = true;
          sendDataToLcd(0);
        }
      }
    }
    if (!PS3.getButtonPress(CIRCLE) && buttonsCIRCLE) {
      command = 'E';
      mySerial.write("E");
      if (DEBUG) {
        Serial.println("Right Motor to original Speed");
      }
      buttonsCIRCLE = false;
      sendDataToLcd(0);
    }

    if (PS3.getButtonClick(SQUARE)) {                                       // pan motor to right
      if (autoAdjust) {
        autoAdjustCount = 4;
        autoAdjustProc(autoAdjustCount);
        if (DEBUG) {
          Serial.println("SPOT 4 - MIDDLE LARGE");
        }
      } else {
        /*
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
        */
        if (lineON) {
          panAdjust = true;
          panCount = analogRead(panPot);
          sendDataToLcd(2);
          if (panCount >= 10) {
            digitalWrite(panMot_1, HIGH);
            digitalWrite(panMot_2, LOW);
            analogWrite(panPwm, panPwmVal);
          }
          startPanVal = analogRead(panPot);
          go = false;
          if (DEBUG) {
            Serial.print("Pan to Left\t");
            Serial.print("Pan : ");
            Serial.println(startPanVal);
          }
        } else {
          command = 'Y';
          mySerial.write("Y");
          if (DEBUG) {
            Serial.println("Slow Left Motor");
          }
          buttonsSQUARE = true;
          sendDataToLcd(0);
        }
      }
    }
    if (!PS3.getButtonPress(SQUARE) && buttonsSQUARE) {
      command = 'U';
      mySerial.write("U");
      if (DEBUG) {
        Serial.println("Left Motor to original Speed");
      }
      buttonsSQUARE = false;
      sendDataToLcd(0);
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
          buttonsL1 = true;
        } else {
          desired_rpm = desired_rpm - rpmIncrementVal;                      // check for invalid rpm values later
          if (desired_rpm != 13 && desired_rpm != 17)
            Serial1.print(desired_rpm);
          if (DEBUG) {
            Serial.print(" RPM set to : ");
            Serial.println(desired_rpm);
          }
        }
      }
      sendDataToLcd(0);
    }
    if (!PS3.getButtonPress(L1) && buttonsL1) {
      autoMode = false;
      command = 'S';
      mySerial.write("S");
      buttonsL1 = false;
      digitalWrite(led_1, LOW);
      if (DEBUG) {
        Serial.println("L1 REALEASED STOPPED");
      }
      sendDataToLcd(0);
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
          buttonsR1 = true;
        } else {
          desired_rpm = desired_rpm + rpmIncrementVal;                      // check for invalid rpm values later
          if (desired_rpm != 13 && desired_rpm != 17)
            Serial1.print(desired_rpm);
          if (DEBUG) {
            Serial.print(" RPM set to : ");
            Serial.println(desired_rpm);
          }
        }
      }
      sendDataToLcd(0);
    }
    if (!PS3.getButtonPress(R1) && buttonsR1) {
      command = 'S';
      mySerial.write("S");
      buttonsR1 = false;
      if (DEBUG) {
        Serial.println("R1 REALEASED STOPPED");
      }
      sendDataToLcd(0);
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
        analogWrite(discPullPwm, 180);
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
      analogWrite(discPullPwm, 180);
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
      sendDataToLcd(0);
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

    if (PS3.getButtonClick(L3)) {                                    // switch controls from chasis to upper bot
      if (lineON == false) {
        lineON = true;
      } else if (lineON == true) {
        lineON = false;
      }
      if (DEBUG) {
        Serial.print("LINE FOLLOWER :"); Serial.println(lineON);
      }
      sendDataToLcd(0);
      loadingON = false;
      motionStat = false;
      panAdjust = false;
      digitalWrite(discMot_1, LOW);
      digitalWrite(discMot_2, LOW);
      analogWrite(discPwm, 0);
      digitalWrite(panMot_1, HIGH);
      digitalWrite(panMot_2, HIGH);
      analogWrite(panPwm, 255);
      digitalWrite(angleMot_1, HIGH);
      digitalWrite(angleMot_2, HIGH);
      analogWrite(anglePwm, 50);
      detachInterrupt(digitalPinToInterrupt(panInterruptPin));
      if (DEBUG) {
        Serial.println("Pan and Disc Motor Stopped, RMCS position reset");
      }
      digitalWrite(discPullMot_1, LOW);
      digitalWrite(discPullMot_2, LOW);
      analogWrite(discPullPwm, 0);
      panCount = 95;
      sendDataToLcd(0);
    }

    if (PS3.getButtonClick(R3)) {
      if (autoAdjust) {
        autoAdjust = false;                                            // start auto adjust
      }
      else {
        autoAdjust = true;
        autoAdjustCount = 0;
      }
      if (DEBUG) {
        Serial.print("autoAdjust : ");
        Serial.println(autoAdjust);
      }
      panCount = analogRead(panPot);
      if (DEBUG) {
        Serial.println(panCount);
      }
      sendDataToLcd(2);
    }
  } else {
    if (stopControl == true) {
      mySerial.write("S");
    }
  }
}
