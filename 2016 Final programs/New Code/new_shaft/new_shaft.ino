#include <PID_v1.h>
//#include <avr/wdt.h>

double Setpoint, Input, Output;
double Kp = 1, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int motorPin = 4;
const int motorPin1 = 2;
const int pwmPin = 3;
const int trigPin = 11;
const int echoPin = 10;
const int shaft_distance = 30;
const int up = A2;
const int down = A3;
bool up1, down1;
int duration, distance;
int pwm;

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
  myPID.SetMode(AUTOMATIC);
  pinMode(up, INPUT);
  pinMode(down, INPUT);
  up1 = down1 = 0;
//  wdt_enable(WDTO_250MS);
}

void measure(void)
{ up1 = digitalRead(up);
  down1 = digitalRead(down);
  Serial.print("up");
  Serial.println(up1);
  Serial.print("down");
  Serial.println(down1);

  digitalWrite (trigPin, LOW);
  delayMicroseconds (2);
  digitalWrite (trigPin, HIGH);
  delayMicroseconds (10);
  digitalWrite (trigPin, LOW);
  duration = pulseIn (echoPin, HIGH);
  distance = (duration / 2) / 29.1;
  Serial.println(distance);
}
void loop() {
//  wdt_reset();
  measure();
  Input = distance;
  myPID.Compute();
  Serial.print("output=");
  Serial.println(Output);
  if (up1 == 0 && down1 == 0)
  {
    if ((distance >= (shaft_distance - 6)) && (distance <= (shaft_distance + 6)))
    { //measure();
      //int pwm =(50 + Output);
      Serial.println("in loop");
      if (distance < (shaft_distance - 1))
      {
        digitalWrite (motorPin, HIGH);
        digitalWrite (motorPin1, LOW);
//        pwm = (45 + Output);
        pwm = (50 + Output);
        analogWrite (pwmPin, pwm);

        Serial.print("up pwm=");
        Serial.println(pwm);
      }
      else if (distance > (shaft_distance + 1))
      {
        digitalWrite (motorPin, LOW);
        digitalWrite (motorPin1, HIGH);
        pwm = (45 - Output);
        analogWrite (pwmPin, pwm);

        Serial.print("Down pwm=");
        Serial.println(pwm);
      }
      else
      { digitalWrite (motorPin, LOW);
        digitalWrite (motorPin1, LOW);
        analogWrite (pwmPin, 0);
      }
    }
    else
    { digitalWrite (motorPin, HIGH);
      digitalWrite (motorPin1, HIGH);
      analogWrite (pwmPin, 0);
    }
  }
  else if (up1 == 1 && down1 == 0)
  { digitalWrite (motorPin, HIGH);
    digitalWrite (motorPin1, LOW);
    analogWrite (pwmPin, 120);
    Serial.print("UP=");
    Serial.println(pwm);
  }
  else if (up1 == 0 && down1 == 1)
  {
    digitalWrite (motorPin, LOW);
    digitalWrite (motorPin1, HIGH);
    analogWrite (pwmPin, 60);
    if (distance < (shaft_distance + 6))
    { digitalWrite (motorPin, LOW);
      digitalWrite (motorPin1, LOW);
      analogWrite (pwmPin, 0);
    }
    Serial.print("DOWN=");
    Serial.println(pwm);
  }
}
