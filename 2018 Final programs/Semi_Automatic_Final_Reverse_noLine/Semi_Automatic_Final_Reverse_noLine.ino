// stop robot at junction
// add watchdog

#define DEBUG 1                         // Serial print on or off in loop()

#include <avr/wdt.h>

#include <SoftwareSerial.h>
SoftwareSerial mySerial(13, 12); // RX, TX

#include <Wire.h>
#include <dcServo.h>
#include <PID_v1.h>

// declare motor objects and provide address of each motor
dcServo FLMot(10);
dcServo LMot(20);
dcServo FRMot(30);
dcServo RMot(40);

#define led_1 A2
#define led_2 A3
#define led_3 A4
#define led_4 A5

char command;
unsigned short Left_Motor_Speed, Right_Motor_Speed;
unsigned short Max_Speed = 70/*250*/, Max_Speed_LF = 100;                             // set max motor rpm and line follower max rpm
short stopDamping = 10, runPWM = 0;                                                         // maximum value is 100
short maxIncSpeed = 120;

//boolean values
boolean lineFollowFront = false, lineFollowReverse = false;                     // run line follow code , forward or reverse
boolean lineON = false;                                                         // decide wheather line follower on or off and invert mode
boolean automaticMode = false, stopControl = false;                             // automatic mode and stopControl for bluetooth disconnection control
boolean jStat = false, jStat_2 = false;                                         // for junction values
boolean setOnLine = false, setOnLineNext = false, solStat = false;                                    // automatic set on centre on line
boolean runSTAT = false, slowForward = false, slowReverse = false, slowForwardStat = false, slowReverseStat = false, slowSTOP = false;
boolean accelerateOnce = false;
boolean spotAdjust = false;
boolean crossedJunction = false;

// PID initial code
double Setpoint, Input, Output;

// Set PID values here
double rpmKp = 0.05, rpmKi = 2.9, rpmKd =  0;                                 // set PID for I2C controlled motors
//double Kp = 7.71, Ki = 2.1, Kd =  0.2;                                        // for Line follower at max speed
//double Kp = 9.71, Ki = 1.9, Kd =  0.2;                                        // awesome PID value for full speed
double Kp = 12.1, Ki = 7.9, Kd =  2.9;                                        // marked
//double Kp = 12.1, Ki = 7.9, Kd = 4.0;
//double slowKp = 2.8, slowKi = 2.9, slowKd =  0.1;                          // for 50 pwm speed
//double slowKp = 7.2, slowKi = 1.9, slowKd =  0.1;                          // ok at 100 pwm speed
double slowKp = 9.2, slowKi = 4.9, slowKd =  2.9;                          // marked
//double slowKp = 9.2, slowKi = 4.9, slowKd =  4.0;
double slowestKp = 3.1, slowestKi = 4.9, slowestKd =  2.9;                  // marked
//double slowestKp = 2.2, slowestKi = 1.9, slowestKd =  3.9;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);                      // declare PID object

// Front LSA08 Pins
#define analogPin A0
#define junctionPulse 3
unsigned short jCount = 0;
int readVal, positionVal, old_val, finalValue;

// Back LSA08 Pins
#define analogPin_2 A1
#define junctionPulse_2 2
unsigned short jCount_2 = 0;
int readVal_2, positionVal_2, old_val_2, finalValue_2;

// line sensor pinmode setting
void setPinmode() {
  pinMode(analogPin, INPUT);
  pinMode(junctionPulse, INPUT);
  pinMode(analogPin_2, INPUT);
  pinMode(junctionPulse_2, INPUT);
}

// line Follower code
void junctionCount() {
  if (digitalRead(junctionPulse) && jStat == false) {
    jStat = true;
  }
  if (jStat == true && !digitalRead(junctionPulse)) {
    jCount++;
    jStat = false;
  }
  /* see this later
    if ((digitalRead(junctionPulse) == 1) && (jStat == false)) {
    jStat = true;
    }
    if ((jStat == true) && (digitalRead(junctionPulse) == 0)) {
    jCount++;
    jStat = false;
    }
    if (jCount == 4) {
    jStat = false;
    jCount = 0;
    runAutomaticMode = false;
    hardStop();
    if (DEBUG) {
      Serial.println("\n Got Junction Motor Stopped : In Loading area");
    }
    }
  */
}

void sensorValue() {
  junctionCount();

  // calculate sensor value
  readVal = analogRead(analogPin);
  positionVal = ((float)readVal / 921) * 70;

  //  Correction Factor
  //if ((positionVal > 60 && old_val < 35) || (positionVal < 10 && old_val > 35)) {
  //  positionVal = old_val;
  //}

  // to see which side bot left the line
  if (positionVal < 70) {
    old_val = positionVal;
  }

  // to set the min and max value to -1 and 71 respectively
  if (positionVal > 70) {
    if (old_val < 30)//15)
      positionVal = -1;
    else if (old_val > 40)//55)
      positionVal = 71;
  }
  if (DEBUG) {
    Serial.print(" Sensor Actual: " );  Serial.print(readVal);
    Serial.print("  Sensor Value: " );  Serial.print(positionVal);
    Serial.print("  Junction : " ); Serial.print(digitalRead(junctionPulse));
    Serial.print("  JCount : " ); Serial.print(jCount);
  }
}


// Line follower code
/************************************************************ Front Sensor Code ********************************************************************/
void lineFollow_frontSensor() {
  sensorValue();
  Input = positionVal;
  if (!digitalRead(junctionPulse)) {
    myPID.Compute();
  }
  if (DEBUG) {
    Serial.print(" Front Sensor PID OUTPUT: " ); Serial.print(Output);
  }
  if (Output == 0) {
    front();
  } else if (Output < 0) {
    Left_Motor_Speed = Max_Speed_LF - abs(Output);
    if (Max_Speed_LF >= 150) {                    // max if left line
      Max_Speed_LF = 175;
    }
    Right_Motor_Speed = Max_Speed_LF;
    if (DEBUG) {
      Serial.print("  Left " );
    }
  } else if (Output > 0) {
    Right_Motor_Speed = Max_Speed_LF - abs(Output);
    if (Max_Speed_LF >= 150) {
      Max_Speed_LF = 175;
    }
    Left_Motor_Speed = Max_Speed_LF;
    if (DEBUG) {
      Serial.print("  Right " );
    }
  }

  FLMot.LineReverse(Left_Motor_Speed);
  LMot.LineReverse(Left_Motor_Speed);
  FRMot.LineReverse(Right_Motor_Speed);
  RMot.LineReverse(Right_Motor_Speed);

  if (Left_Motor_Speed > Right_Motor_Speed) {
    runPWM = Left_Motor_Speed;
  } else {
    runPWM = Right_Motor_Speed;
  }
  if (DEBUG) {
    Serial.print("  Left Motor Speed : " ); Serial.print(Left_Motor_Speed);  Serial.print("  Right Motor Speed : " );  Serial.println(Right_Motor_Speed);
  }
}

void junctionCount_2() {                                                          // seee when running automatic mode
  if (digitalRead(junctionPulse_2) && jStat_2 == false) {
    jStat_2 = true;
  }
  if (jStat_2 == true && !digitalRead(junctionPulse_2)) {
    jCount_2++;
    jStat_2 = false;
  }
}

void sensorValue_2() {
  junctionCount_2();
  // calculate sensor value
  readVal_2 = analogRead(analogPin_2);
  positionVal_2 = ((float)readVal_2 / 921) * 70;

  //  Correction Factor cheack it
  //if ((positionVal_2 > 55 && old_val_2 < 35) || (positionVal_2 < 15 && old_val_2 > 35)) {
  //  positionVal_2 = old_val_2;
  //}

  // to see which side bot left the line
  if (positionVal_2 <= 70)
    old_val_2 = positionVal_2;

  // to set the min and max value to 0 and 70 respectively
  if (positionVal_2 > 70) {
    if (old_val_2 < 30) { //15) {
      positionVal_2 = -1;
    } else if (old_val_2 > 40) { // 55) {
      positionVal_2 = 71;
    }
  }
  if (DEBUG) {
    Serial.print(" Sensor Actual: " );  Serial.print(readVal_2);
    Serial.print("  Sensor Value: " );  Serial.print  (positionVal_2);
    Serial.print("  Junction : " ); Serial.print(digitalRead(junctionPulse_2));
    Serial.print("  JCount : " ); Serial.print(jCount_2);
  }
}

/************************************************************ Back Sensor Code ********************************************************************/
void lineFollow_backSensor() {
  //  if (adjustOnLine == false) {                                      // for automatic mode
  sensorValue_2();
  Input = positionVal_2;
  if (!digitalRead(junctionPulse_2)) {
    myPID.Compute();
  }
  if (DEBUG) {
    Serial.print("  Back Sensor PID OUTPUT: " ); Serial.print(Output);
  }
  if (Output == 0) {
    back();
  } else if (Output < 0) {
    Right_Motor_Speed = Max_Speed_LF - abs(Output);
    Left_Motor_Speed = Max_Speed_LF;
    if (DEBUG) {
      Serial.print("  Left " );
    }
  } else if (Output > 0) {
    Left_Motor_Speed = Max_Speed_LF - abs(Output);
    Right_Motor_Speed = Max_Speed_LF;
    if (DEBUG) {
      Serial.print("  Right " );
    }
  }

  FLMot.LineForward(Left_Motor_Speed);
  LMot.LineForward(Left_Motor_Speed);
  FRMot.LineForward(Right_Motor_Speed);
  RMot.LineForward(Right_Motor_Speed);

  if (Left_Motor_Speed > Right_Motor_Speed) {
    runPWM = Left_Motor_Speed;
  } else {
    runPWM = Right_Motor_Speed;
  }
  if (DEBUG) {
    Serial.print("  Left Motor Speed : " );  Serial.print(Left_Motor_Speed);  Serial.print("  Right Motor Speed : " );  Serial.println(Right_Motor_Speed);
  }
}

void front() {                                                  // forward motion of the bot
  maxIncSpeed = Max_Speed;
  if (runPWM < maxIncSpeed) {
    forwardPWM(runPWM);
    runPWM++;
    delayMicroseconds(8000);
  } else {
    FLMot.LineReverse(Max_Speed);
    LMot.LineReverse(Max_Speed);
    FRMot.LineReverse(Max_Speed);
    RMot.LineReverse(Max_Speed);
    if (DEBUG) {
      Serial.print("  GOING FRONT" );
      Serial.print("  Speed : " );
      Serial.println(Max_Speed);
    }
    command = -1;
  }
}

void back() {                                                   // reverse motion of the bot
  maxIncSpeed = Max_Speed;
  if (runPWM < maxIncSpeed) {
    reversePWM(runPWM);
    runPWM++;
    delayMicroseconds(8000);
  } else {
    FLMot.LineForward(Max_Speed);
    LMot.LineForward(Max_Speed);
    FRMot.LineForward(Max_Speed);
    RMot.LineForward(Max_Speed);
    if (DEBUG) {
      Serial.print("  GOING BACK" );
      Serial.print("  Speed : " );
      Serial.println(Max_Speed);
    }
    command = -1;
  }
}

void rightSide() {                                              // turn the bot to right
  maxIncSpeed = Max_Speed;
  if (runPWM < maxIncSpeed) {
    FLMot.LineReverse(runPWM);
    LMot.LineReverse(runPWM);
    runPWM++;
    delayMicroseconds(8000);
    if(DEBUG){
      Serial.print("Right PWM : ");
      Serial.println(runPWM);
    }
  } else {
    FLMot.LineReverse(Max_Speed);
    LMot.LineReverse(Max_Speed);
    //  FRMot.Forward(Max_Speed);
    //  RMot.Forward(Max_Speed);
    if (DEBUG) {
      Serial.print("  GOING LEFT" );
      Serial.print("  Speed : " );
      Serial.println(Max_Speed);
    }
    command = -1;
  }
}

void leftSide() {                                               // turn the bot to left
  maxIncSpeed = Max_Speed;
  if (runPWM < maxIncSpeed) {
    FRMot.LineReverse(runPWM);
    RMot.LineReverse(runPWM);
    if (DEBUG) {
      Serial.print("  GOING RIGHT" );
      Serial.print("  Speed : " );
      Serial.println(runPWM);
    }
    runPWM++;
    delayMicroseconds(8000);
  } else {
    FRMot.LineReverse(Max_Speed);
    RMot.LineReverse(Max_Speed);
    if (DEBUG) {
      Serial.print("  GOING RIGHT" );
      Serial.print("  Speed : " );
      Serial.println(Max_Speed);
    }
    command = -1;
  }
}

void rightTurn() {                                                  // nothing here left out
  if (DEBUG) {
    Serial.print("  No action here GOING RIGHT TURN" );
    Serial.print("  Speed : " );
    Serial.println(Max_Speed);
  }
}

void leftTurn() {                                                   // nothing here just left out
  if (DEBUG) {
    Serial.print("No action here  GOING LEFT TURN" );
    Serial.print("  Speed : " );
    Serial.println(Max_Speed);
  }
}

void rightTurn_2() {                                               // nothing here left out
  if (DEBUG) {
    Serial.print("No action here  GOING RIGHT TURN INVERT" );
    Serial.print("  Speed : " );
    Serial.println(Max_Speed);
  }
}

void leftTurn_2() {                                                 // nothing here left out
  if (DEBUG) {
    Serial.print("No action here  GOING LEFT TURN INVERT" );
    Serial.print("  Speed : " );
    Serial.println(Max_Speed);
  }
}

void forwardPWM(int s) {
  FLMot.LineReverse(s);
  LMot.LineReverse(s);
  FRMot.LineReverse(s);
  RMot.LineReverse(s);
  if (DEBUG) {
    Serial.print("Forward PWM : ");
    Serial.println(s);
  }
}

void reversePWM(int s) {
  FLMot.LineForward(s);
  LMot.LineForward(s);
  FRMot.LineForward(s);
  RMot.LineForward(s);
  if (DEBUG) {
    Serial.print("Reverse PWM : ");
    Serial.println(s);
  }
}

void setOnSpotCaller(int le, int ec, int s) {
  FLMot.setOnSpot(le, ec, s);
  LMot.setOnSpot(le, ec, s);
  FRMot.setOnSpot(le, ec, s);
  RMot.setOnSpot(le, ec, s);
  if (DEBUG) {
    Serial.print("Setting on Spot");
    Serial.print("\tLine End : ");
    Serial.print(le);
    Serial.print("  End : ");
    Serial.print(ec);
    Serial.print("  Speed : ");
    Serial.println(s);
  }
}

void normalStop(int d) {                                            // stop all the motors
  if (d == 2) {
    command = -1;
    FLMot.Stop(1);
    LMot.Stop(1);
    FRMot.Stop(1);
    RMot.Stop(1);
  } else {
    if (runPWM > -15 && d != 1) {
      if (slowForwardStat) {
        if (runPWM < 0) {
          FLMot.LineForward(abs(runPWM));
          LMot.LineForward(abs(runPWM));
          FRMot.LineForward(abs(runPWM));
          RMot.LineForward(abs(runPWM) );
        }
        else {
          FLMot.LineReverse(abs(runPWM));
          LMot.LineReverse(abs(runPWM));
          FRMot.LineReverse(abs(runPWM));
          RMot.LineReverse(abs(runPWM));
        }
        if (DEBUG) {
          Serial.print("runPWM : ");
          Serial.println(runPWM);
        }
      }
      else if (slowReverseStat) {
        if (runPWM < 0)
          forwardPWM(abs(runPWM));
        else
          reversePWM(abs(runPWM));
      }
      runPWM--;
      delayMicroseconds(50);
    } else {
      slowReverseStat = false;
      slowForwardStat = false;
      runPWM = 0;
      if (d == 1) {
        FLMot.Stop(10);
        LMot.Stop(d);
        FRMot.Stop(10);
        RMot.Stop(d);
      } else {
        FLMot.Stop(d);
        LMot.Stop(d);
        FRMot.Stop(d);
        RMot.Stop(d);
      }
      if (DEBUG) {
        Serial.println(" NORMAL STOP" );
      }
      if (command != 'P') {
        if (command != 'O') {
          command = -1;
        }
      }
    }
  }
}

void setRpmPID(double rpmKp, double rpmKi, double rpmKd) {              // set auto RPM adjustment PID values
  FLMot.setPID(rpmKp, rpmKi, rpmKd);
  LMot.setPID(rpmKp, rpmKi, rpmKd);
  FRMot.setPID(rpmKp, rpmKi, rpmKd);
  RMot.setPID(rpmKp, rpmKi, rpmKd);
  if (DEBUG) {
    Serial.print(" PID values set to " );
    Serial.print(" :- Kp : ");
    Serial.print(rpmKp);
    Serial.print(" Ki : ");
    Serial.print(rpmKi);
    Serial.print(" Kd : ");
    Serial.print(rpmKd);
  }
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600);

  Serial.println(" PROGRAM STARTED ");

  setPinmode();
  Serial.println("Line Sensor Pins set ");
  Left_Motor_Speed = Right_Motor_Speed = Max_Speed_LF;
  Serial.print("Motor speed set to "); Serial.println(Max_Speed);
  Serial.print("Line Follower speed set to "); Serial.println(Max_Speed_LF);

  Setpoint = 35;                                                      // maintain line sensor to the value of 35
  Input = 36;                                                         // initial input of PID
  myPID.SetMode(AUTOMATIC);                                           // start the PID

  pinMode(led_1, OUTPUT);
  pinMode(led_2, OUTPUT);
  pinMode(led_3, OUTPUT);
  pinMode(led_4, OUTPUT);

  normalStop(10);
  wdt_enable(WDTO_1S);

  /*
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

// Forward : F
// Reverse : B
// Left : L
// Right : R
// Stop : S
// Hard Stop : H
// LineFollow Forward : A
// LineFollow Backward : Z
// Slow Left : O
// Slow Right : P
// Position left : V
// Position Right : B
// Automatic mode : Q
// Automode to Loading : D
// Change the Line Follower Speed : G

void loop() {
  wdt_reset();

  if (mySerial.available()) {
    command = mySerial.read();
    if (DEBUG) {
      Serial.print("Command : ");
      Serial.println(command);
    }
  }

  if (command == 'F') {
    slowForwardStat = true;
    front();
  } else if (command == 'B') {
    slowReverseStat = true;
    back();
  } else if (command == 'L') {
    rightSide();
  } else if (command == 'R') {
    leftSide();
  }

  if (command == 'S') {
    normalStop(10);
    digitalWrite(led_4, LOW);
    automaticMode = false;
    accelerateOnce = false;
    spotAdjust = false;
  }
  if (command == 'H') {
    normalStop(1);
    command = -1;
    accelerateOnce = false;
  }

  if (command == 'D') {
    slowForwardStat = true;
    maxIncSpeed = 150;
    if (runPWM < maxIncSpeed && !accelerateOnce) {
      forwardPWM(runPWM);
      runPWM++;
      delayMicroseconds(4000);
    } else {
      accelerateOnce = true;
      myPID.SetTunings(Kp, Ki, Kd);
      //myPID.SetOutputLimits(-110, 110);
      myPID.SetOutputLimits(-130, 130);
      Max_Speed_LF = 150;
      lineFollow_frontSensor();
    }
  }

  if (command == 'G') {
    digitalWrite(led_4, HIGH);
    Max_Speed_LF = 70;
    myPID.SetTunings(slowestKp, slowestKi, slowestKd);
    myPID.SetOutputLimits(-50, 50);
    lineFollow_frontSensor();
    if (DEBUG) {
      Serial.println("Slow Line Follower started");
    }
  }

  if (command == 'A') {
    slowForwardStat = true;
    maxIncSpeed = 100;
    if (runPWM < maxIncSpeed && !accelerateOnce) {
      forwardPWM(runPWM);
      runPWM++;
      delayMicroseconds(4000);
    } else {
      accelerateOnce = true;
      Max_Speed_LF = 100;
      myPID.SetTunings(slowKp, slowKi, slowKd);
      myPID.SetOutputLimits(-80, 80);
      lineFollow_frontSensor();
    }
  } else if (command == 'Z') {
    slowReverseStat = true;
    maxIncSpeed = 100;
    if (runPWM < maxIncSpeed && !accelerateOnce) {
      reversePWM(runPWM);
      runPWM++;
      delayMicroseconds(4000);
    } else {
      accelerateOnce = true;
      Max_Speed_LF = 100;
      myPID.SetTunings(slowKp, slowKi, slowKd);
      myPID.SetOutputLimits(-80, 80);
      lineFollow_backSensor();
    }
  }

  if (command == 'O') {
    if (!spotAdjust) {
      slowReverseStat = true;
      maxIncSpeed = 70;
      if (runPWM < maxIncSpeed && !accelerateOnce) {
        reversePWM(runPWM);
        runPWM++;
        delayMicroseconds(4000);
        jCount_2 = 0;
        crossedJunction = false;
      } else {
        accelerateOnce = true;
        Max_Speed_LF = 70;
        myPID.SetTunings(slowestKp, slowestKi, slowestKd);
        myPID.SetOutputLimits(-50, 50);
        lineFollow_backSensor();
        if (jCount_2 == 1) {
          jCount_2 = 0;
          spotAdjust = true;
          normalStop(1);
        }
      }
    }
  } else if (command == 'P') {
    if (!spotAdjust) {
      slowForwardStat = true;
      maxIncSpeed = 70;
      if (runPWM < maxIncSpeed && !accelerateOnce) {
        forwardPWM(runPWM);
        runPWM++;
        delayMicroseconds(4000);
        jCount = 0;
      } else {
        accelerateOnce = true;
        myPID.SetTunings(slowestKp, slowestKi, slowestKd);
        myPID.SetOutputLimits(-50, 50);
        Max_Speed_LF = 70;
        lineFollow_frontSensor();
      }
      if (jCount == 1) {
        jCount = 0;
        spotAdjust = true;
        normalStop(1);
      }
    }
  }
  // for position on encoder
  if (command == 'V') {
    int relPos = 2, speeed = 40;
    FLMot.Position(1, relPos, speeed);
    LMot.Position(1, relPos, speeed);
    FRMot.Position(1, relPos, speeed);
    RMot.Position(1, relPos, speeed);
    if (DEBUG) {
      Serial.println("Move by Count to Left");
    }
    command = -1;
  } else if (command == 'N') {
    int relPos = 2, speeed = 40;
    FLMot.Position(0, relPos, speeed);
    LMot.Position(0, relPos, speeed);
    FRMot.Position(0, relPos, speeed);
    RMot.Position(0, relPos, speeed);
    if (DEBUG) {
      Serial.println("Move by Count to Right");
    }
    command = -1;
  }

  if (command == 'Q') {
    slowReverseStat = true;
    Max_Speed_LF = 100;
    myPID.SetTunings(slowKp, slowKi, slowKd);
    myPID.SetOutputLimits(-80, 80);
    automaticMode = true;
    command = -1;
    jCount_2 = 0;
    digitalWrite(led_1, LOW);
    digitalWrite(led_2, LOW);
    digitalWrite(led_3, LOW);
  }
  if (automaticMode) {                                                          // Automatic bot line follow code here
    maxIncSpeed = 100;
    if (runPWM < maxIncSpeed && !accelerateOnce) {
      reversePWM(runPWM);
      runPWM++;
      delayMicroseconds(4000);
    } else {
      accelerateOnce = true;
      lineFollow_backSensor();                                                    // call to reverse line follower code
      if (DEBUG)
        Serial.println("Going Automatic checking Junction");
      if (jCount_2 == 1) {
        digitalWrite(led_1, HIGH);
      }
      if (jCount_2 == 2) {
        if (DEBUG)
          Serial.print("Stopped on JCount 3");
        jCount_2 = 0;
        automaticMode = false;
        normalStop(1);
        //delay(100);
        setOnLine = true;
        Max_Speed_LF = 40;
        myPID.SetTunings(slowestKp, slowestKi, slowestKd);
        myPID.SetOutputLimits(-30, 30);
        digitalWrite(led_2, HIGH);
      }
    }
  }
  if (setOnLine) {                                                              //auto adjustment of the bot on centre line
    /*
      Max_Speed = 20;
      reversePWM(Max_Speed);
      if (DEBUG) {
      Serial.print("Running at speed");
      Serial.println(Max_Speed);
      }
    */
    lineFollow_backSensor();
    if (digitalRead(junctionPulse_2)) {
      digitalWrite(led_3, HIGH);
      setOnLine = false;
      setOnLineNext = true;
      normalStop(1);
      if (DEBUG) {
        Serial.println("Stopped at Center Junction");
      }
      digitalWrite(led_1, LOW);
      digitalWrite(led_2, LOW);
      /*
        delay(500);
        int relPos = 3, speeed = 40;
        FLMot.Position(0, relPos, speeed);
        LMot.Position(0, relPos, speeed);
        FRMot.Position(0, relPos, speeed);
        RMot.Position(0, relPos, speeed);
      */
    }
  }
  if (setOnLineNext) {
    //setOnSpotCaller(0, 13, 45);
    //reversePWM(40);
    //    int relPos = 3, speeed = 35;
    //    FLMot.Position(0, relPos, speeed);
    //    LMot.Position(0, relPos, speeed);
    //    FRMot.Position(0, relPos, speeed);
    //    RMot.Position(0, relPos, speeed);
    setOnLineNext = false;
    //    delay(500);
    digitalWrite(led_3, LOW);
  }
}
