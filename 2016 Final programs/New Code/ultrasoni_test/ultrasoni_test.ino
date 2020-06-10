#include <Servo.h>
Servo myservo;

int trig = 7;
int echo = 6;
int set, sett;
long duration_front;
int distance_front;
int count;
int led1 = 50, led2 = 53, led3 = 12;
bool t = 0;
const int angle = 90;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(13, OUTPUT);
  set = 0;
  sett = 0;
  count  = 0;
  pinMode(led1 , OUTPUT);
  pinMode(led2 , OUTPUT);
  pinMode(led3 , OUTPUT);
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
  myservo.attach(10);
  t = 0;
  myservo.write(angle);
}

void loop() { 
  // put your main code here, to run repeatedly:
  calculate();
  Serial.print("Distance front =");
  Serial.println(distance_front);
  myservo.write(angle);
  valset();
  if (set == 1) {
    digitalWrite(13, LOW);
    sett = 1;
  }
  if (sett == 1 && set == 0) {
    digitalWrite(13, HIGH);
    sett = 2;
    while (1) {
      calculate();
      if (distance_front > 40) {
        break;
      }
    }
  }
  valset();
  if (sett == 2 && set == 1) {
    count++;
    delay(0);
    //    digitalWrite(13, LOW);
  }
  if (count == 2) {
    turn();
  }
  led();
  Serial.print("Count:");
  Serial.println(count);
}
void calculate() {

  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);                                      //  ultrasonic sending signal
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration_front = pulseIn(echo, HIGH);                         // calculating distance in cm
  distance_front = (duration_front / 2) / 29.1;

}
void valset() {

  if (distance_front > 40) {
    set = 1;
  }
  if (distance_front <= 40) {
    set = 0;
  }
}
void led() {
  if (count == 1) {
    digitalWrite(led1, HIGH);
  }
  if (count == 2) {
    digitalWrite(led2, HIGH);
  }
  if (count == 3) {
    digitalWrite(led3, HIGH);
  }
}
void turn() {
  if (t == 0) {
    for (int i = angle; i >= 60; i--) {
      myservo.write(i);
    }
    delay(250);
    myservo.write(angle);
    t = 1;
  }
}
