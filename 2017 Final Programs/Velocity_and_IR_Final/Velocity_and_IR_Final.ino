// send rpm and pwm on request - added - test it

#define DEBUG 1

#include <SoftwareSerial.h>
SoftwareSerial mySerial(7, 6); // RX, TX

#include <PID_v1.h>
double Setpoint, Input, Output;
//double Kp = 0.025, Ki = 0.08, Kd = 0;                       // nice adjustment on 3000 rpm

double Kp = 0.09, Ki = 0.25, Kd = 0;                       // nice adjustment on 3000 rpm

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

#include <StopWatch.h>
StopWatch MySW(StopWatch::MICROS);

#define interruptPin 3

#define DIR 10
#define PWM 9

double tym = 1;
double desired_rpm = 4200;
double rpm, rps;
String RPM;
int counter = 0, setRpm;

unsigned long Time, startTime, endTime;
double d = 200;
int total = 0;
boolean startP =  false, endP =  false;
char command = 0;
boolean motorON = false;

String str;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);

  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), incrementCounter, RISING);
 
  Serial.println("Program Started");

  Serial.print("Set PID for Throwing motor | DesiredRPM : ");
  Serial.println(desired_rpm);
  Input = 0;
  Setpoint = desired_rpm;
  myPID.SetMode(AUTOMATIC);
  myPID.SetControllerDirection(DIRECT);
  myPID.SetOutputLimits(50, 105);

  digitalWrite(DIR, HIGH);                              // set Throwing motor direction
  analogWrite(PWM, 0);                                  // stop the motor
}

void loop() {
  if (mySerial.available()) {
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    setRpm = mySerial.parseInt();
    mySerial.flush();
    if (DEBUG)
      Serial.println(setRpm);
    if (setRpm == 17) {
      analogWrite(PWM, 40);
      motorON = true;
      if (DEBUG) {
        Serial.println("Motor Started");
      }
    } else if (setRpm == 13) {
      analogWrite(PWM, 0);
      motorON = false;
      if (DEBUG) {
        Serial.println("Motor Stopped");
      }
    /*} else if (setRpm == -5) {
      mySerial.print(str + rpm + "," + Output + ",");
      //mySerial.flush();
      if(DEBUG){
        Serial.print("Requested data is sent");
      }*/
    } else {
      if (motorON) {
        if (setRpm >= 1500 && setRpm <= 6000)
          Setpoint = setRpm;
        if (DEBUG) {
          Serial.print("RPM set to ");
          Serial.println(Setpoint);
        }
      }
    }
    counter = 0;
    attachInterrupt(digitalPinToInterrupt(interruptPin), incrementCounter, RISING);
  }
  if (Serial.available()) {
    setRpm = Serial.parseInt();
    if (DEBUG)
      Serial.println(setRpm);
    if (setRpm >= 1500 && setRpm <= 6000)
      Setpoint = setRpm;
  }
  if (motorON) {
    if (rpm > 1000 && rpm <= 6000)
      Input = rpm;
    myPID.Compute();
    if (!DEBUG) {
      //Serial.print(" \tOUTPUT : ");
      //Serial.println(Output);
    }
    analogWrite(PWM, Output);
  }
}

void incrementCounter() {
  counter ++;
  //  Serial.print("Count : ");
  //  Serial.println(counter);
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  if (counter == 1)
    MySW.start();
  if (counter >= 2) {
    tym = MySW.elapsed();
    rpm = (60000000 / tym) ;
    if (DEBUG) {
      Serial.print("\t RPM : ");
      Serial.println(rpm);
      //Serial.print("\t Output : ");
      //Serial.println(Output);
    }
    counter = 0;
    MySW.reset();
  }
  attachInterrupt(digitalPinToInterrupt(interruptPin), incrementCounter, RISING);
}
