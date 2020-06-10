#include <PID_v1.h>
#include <Servo.h>
#define PIN_INPUT 0
#define PIN_OUTPUT 3

double Setpoint, Input, Output;
Servo myservo;
int pos = 0;
double Kp = 1, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
char a;
double b = 0;
int c = 0;
int last_pos = 90;
int sensor = A0;
int prox = 6;
int swt = 11;
bool stat;
const int junctionPulse = 13;
const int fixangle = 90;

void setup()
{
  //initialize the variables we're linked to
  pinMode(10, OUTPUT);
  Serial.begin(9600);
  myservo.attach(10);  // attaches the servo on pin 9 to the servo object
  pinMode(sensor, INPUT);
  pinMode(prox, INPUT);
  pinMode(swt, INPUT);
  pinMode(12, OUTPUT);
  pinMode(junctionPulse, INPUT);

  Input = 35;
  Setpoint = 35;


  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-50, 50);
  for (int i = 0; i <= 85; i++)
  {
    last_pos = i;
    myservo.write(i);
    // Serial.println(i);
    delay(15);
  }
  myservo.write(85);
}

int angle, temp;
void loop()
{
  int s = digitalRead(swt);
  int temp = analogRead(sensor);
  temp = map(temp, 0, 1023, 0, 255);

  temp = ((temp * 0.02) / 4.5) * 70;
  //  while (Serial.available() > 0) {
  Serial.print("original:");
  Serial.println(temp);
  if (temp >= 65) {
    if (angle > 90) {
      temp = 65;
    }
    else if (angle < 90) {
      temp = 0;
    }
  }
  if (angle >= 80 && angle <= 100 && (temp == 65 || temp == 0)) {
    temp = 35;
    Serial.println("runned");
    while (1) {
      myservo.write(90);
      temp = analogRead(sensor);
      temp = map(temp, 0, 1023, 0, 255);
      temp = ((temp * 0.02) / 4.5) * 70;
      if(temp >= 0 && temp < 65){
        break;
      }
    }
  }
  Serial.println(temp);
  Input = ceil (temp);
  myPID.Compute();

  angle = 90 - Output;
  Serial.print("angle:");
  Serial.println(angle);
  myservo.write(angle);
  delay(10);
  //  }
  stat = digitalRead(junctionPulse);
  if (stat == HIGH && s == 0) {
    //    Serial.println("Junction Turn");
    angle = 135;
    myservo.write(angle);
    delay(1000);
    myservo.write(fixangle);
  }
  //***************************river****************************
  int set1 = 0;
  while (s == 1)
  { int p = digitalRead(prox);
    myservo.write(fixangle);
    while (p == 1 && set1 == 0)
    { p = digitalRead(prox);
      if (p == 0)
      { set1 = 1;
        break;
      }
    }
    while (p == 1 && set1 == 1)
    { p = digitalRead(prox);
      if (p == 0)
      { set1 = 2;
        break;
      }
    }
    if (set1 == 2)
    {
      for (int i = fixangle; i >= 70; i--) {
        myservo.write(i);                    // sets the servo position according to the scaled value
        delay(15);                           // waits for the servo to get there
      }
      delay(95);
      myservo.write(fixangle);
      delay(1000);
      break;
    }
  }
  //******************************************************
}
