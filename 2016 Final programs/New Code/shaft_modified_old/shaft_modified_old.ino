#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 1, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int motorPin = 10;
const int motorPin1 = 12;
const int pwmPin = 11;
const int trigPin = 5;
const int echoPin = 4;

const int shaft_distance = /*21 22 11 23 29*/30;

const int up = A5;
const int down = A4;
bool up1, down1;

void setup() {

  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  Input = shaft_distance;
  Setpoint = shaft_distance;
  myPID.SetOutputLimits(-100, 100);
  //SetSampleTime()
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  pinMode(up1, INPUT);
  pinMode(down1, INPUT);
  up1 = down1 = 0;
}
void loop() {
  int duration, distance;
  digitalWrite (trigPin, LOW);
  delayMicroseconds (2);
  digitalWrite (trigPin, HIGH);
  delayMicroseconds (10);
  digitalWrite (trigPin, LOW);
  duration = pulseIn (echoPin, HIGH);
  distance = (duration / 2) / 29.1;
  Serial.println(distance);
  //  if (distance >= 26 && distance <= 32)
  //if(distance>=25&&distance<=32)
  //  {
  Input = distance;
  // Serial.println("Input");
  myPID.Compute();

  up1 = digitalRead(up);
  down1 = digitalRead(down);

  if (up1 == HIGH) {
    digitalWrite (motorPin, LOW);
    digitalWrite (motorPin1, HIGH);
    analogWrite(pwmPin, 80);
  }
  else if (down1 == HIGH) {
    digitalWrite (motorPin1, LOW);
    digitalWrite (motorPin, HIGH);
    analogWrite(pwmPin, 80);
  }
  else {
    digitalWrite (motorPin, LOW);
    digitalWrite (motorPin1, LOW);
    analogWrite(pwmPin, 0);
  }
  // Serial.print("output=");
  //Serial.println(Output);
  int pwm;
  if (up1 == LOW || down1 == LOW) {
    // Code to protect from violation
    if (distance == 0) {
      distance = shaft_distance;
    }
    /*    if (distance < 10) {
          distance = shaft_distance;
        }
    */
    if (distance >= 26 && distance <= 32) {
      // if(distance>=25&&distance<=32)
      if (distance > (shaft_distance + 1))
      {
        digitalWrite (motorPin, HIGH);
        digitalWrite (motorPin1, LOW);
        int pwm = (45 - Output);
        analogWrite (pwmPin, pwm);
        Serial.print(" Up pwm=");
        Serial.println(pwm);
      }
      else if (distance < (shaft_distance - 1))
      {
        digitalWrite (motorPin, LOW);
        digitalWrite (motorPin1, HIGH);
        int pwm = 2 * (50 + Output);
        analogWrite (pwmPin, pwm);

        Serial.print("Down pwm=");
        Serial.println(pwm);
      } else {
        digitalWrite (motorPin, LOW);
        digitalWrite (motorPin1, LOW);
        for (int i = 255; i >= 0; i = i - 5) {
          analogWrite (pwmPin, 0);
          delay(1);
        }
        // Serial.println("stop");
      }
    } else {
      digitalWrite (motorPin, LOW);
      digitalWrite (motorPin1, LOW);
      analogWrite(pwmPin, 0);
    }
    if (distance < shaft_distance) {
      digitalWrite (motorPin, LOW);
      digitalWrite (motorPin1, LOW);
      analogWrite (pwmPin, 0);
      Serial.println("Stop");
    }
  }
}
