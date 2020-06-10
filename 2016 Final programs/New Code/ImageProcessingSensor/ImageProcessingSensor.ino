#include <PID_v1.h>                                                               // include PID library
#include <Servo.h>                                                                // include servo library
Servo myservo;                                                                    // declare servo object

int trig = 10;//6;                                                                // all pins declared here
int echo = 9;//7;
int led1 = 11, led2 = 12, led3 = 0;

int set, sett;                                                                    // all variables declared here
long duration_front;
int distance_front;
int count;

bool t = 0;
const int angle = 93;

// PID values for Sensor
double Setpoint, Input, Output;
double Kp = 1, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);                             // myPID object declared for Sensor

// PID values for Image processing
double Setpoint1, Input1, Output1;
double Kp1 = 20, Ki1 = 0, Kd1 = 0;
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);                       // myPID1 object declared for Image processing


// Variables for Image processing
int pos = 0;                                                                          // variable to store the servo position
char a;
double b = 0;
int c = 0;
int last_pos = 90;

// declared pins for Sensor
int sensor = A1;//A0;
int swt = 3;//5;
bool stat;
const int junctionPulse = A2;//13;
int change = 0;                                                                       // pin number used to take button input of shift betn sensor and image processing

bool debug = 0;                                                                        // to turn on and off Serial
bool shift = 0;                                                                        // to switch between sensor and image processing

void setup() {
  // put your setup code here, to run once:
  if (debug == 1) {
    Serial.begin(9600);                                                                 // begin Serial communication
  }
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(13, OUTPUT);
  set = 0;
  sett = 0;
  count  = 0;
  pinMode(led1 , OUTPUT);
  pinMode(led2 , OUTPUT);
  pinMode(led3 , OUTPUT);
  
  digitalWrite(led1, HIGH);                                                             // led on ,leds used for air push using Pneumatic sensing the island
  digitalWrite(led2, HIGH);
  digitalWrite(led3, HIGH);
  
  myservo.attach(A0);//10);
  t = 0;
  
  pinMode(sensor, INPUT);
  pinMode(swt, INPUT);
  pinMode(change, INPUT);
  pinMode(12, OUTPUT);
  pinMode(junctionPulse, INPUT);

  Input = 33;                                                                                   // Values for Sensor
  Setpoint = 33;
  
  Input1 = 1.6;                                                                                  // Values for Image Processing
  Setpoint1 = 1.6;

  //turn the PID on
  myPID.SetOutputLimits(-35, 35);
  myPID.SetMode(AUTOMATIC);                                                                     // sensor

  myPID1.SetOutputLimits(-50, 50);
  myPID1.SetMode(AUTOMATIC);                                                                    // image processing

  for (int i = 0; i <= angle; i++)                                                              // turn the servo from 0 to 90 in starting
  {
    myservo.write(i);
    delay(15);
  }
  
  myservo.write(angle);
  
  ledoff();                                                                                     // turn off the led's
  shift = 0;                                                                                    // variable used to shift betweenSensor and Image processing
}

int temp1, angle1;

void loop() {
  // put your main code here, to run repeatedly:
  int r = digitalRead(change);                                      // read button for shifting betn sensor and image proessing
  if (r == 1) {                                                     // change form sensor to Image proscessing and vice versa
    if (shift == 0)
      shift = 1;
    else if (shift == 1)
      shift = 0;
  }
  if (shift == 0) {                                                               // run the Sensor code
    sensor_code();
  }
  else if (shift == 1) {                                                          // run the image processing code
    image_processing();
  }
//  river_crossing();                                                               // river code , not needed now EDF is working well
}

void sensor_code() {
  int temp = analogRead(sensor);                                                    // read the sensor AN pin values
  temp = map(temp, 0, 1023, 0, 255);
  temp = ((temp * 0.02) / 4.5) * 70;                                                // formula for calculating the values
  
  if (debug == 1) {
    Serial.print("original:");
    Serial.println(temp);
  }
  
  if (temp > 70) {                                                               // this code used to keep the castor turned correctly if line is missed
    if (angle1 > 90) {
      temp = 70;
    }
    else if (angle1 < 90) {
      temp = 0;
    }
  }
  
  if (angle1 >= 80 && angle1 <= 100 && (temp == 70 || temp == 0)) {              // this code used to keep the castor straight if sensor misses the values on line
    temp = 35;
    while (1) {
      myservo.write(angle);                                                      // castor is straight
      temp = analogRead(sensor);
      temp = map(temp, 0, 1023, 0, 255);
      temp = ((temp * 0.02) / 4.5) * 70;
      if (temp >= 0 && temp <= 70) {                                              // if sensor detects again
        break;
      }
    }
  }
  
  if (debug == 1) {
    Serial.println(temp);
  }
  
  Input = ceil (temp);                                                            // take the value from sensor and give it to PID Input
  myPID.Compute();                                                                // compute the output

  angle1 = 90 - Output;                                                           // set the angle according to the output
  
  if (debug == 1) {
    Serial.print("angle:");
    Serial.println(angle1);
  }
  
  myservo.write(angle1);                                                          // turn the castor accordingly
  delay(10);

  stat = digitalRead(junctionPulse);                                              // read the senosor JN pin
  
  if (debug == 1) {
    Serial.print("Junction:");
    Serial.println(stat);
  }
                                                        
  if (stat == HIGH) {                                                               // code to turn the castor on Hill3 junction
    if (debug == 1) {
      Serial.println("Junction Turn");
    }
    angle1 = 135;                                                                     // angle to turn on junction turn
    myservo.write(angle1);
    delay(1000);
    myservo.write(angle);
  }
}

void image_processing() {
  Setpoint = 1.6;
  while (Serial.available() > 0) {
    b = Serial.parseInt();
    if (b == 999)
    {
      myservo.write(150);
      delay(50);
    }
    else
    { if (debug == 1) {
        Serial.print("b :");
        Serial.println(b);
      }
      //delay(500);
      //b=Serial.read();
      Input = b / 100;
      if (debug == 1) {
        Serial.println(b);
      }
      myPID.Compute();
      if (debug == 1) {
        Serial.print("Output");
        Serial.println(Output);
      }
      b = 0;
      temp1 = 90 - Output;
      if (debug == 1) {
        Serial.println(temp1);
      }
      myservo.write(temp1);
    }
    delay(10);
    if (debug == 1) {
      Serial.print("temp");
      Serial.println(temp1);
    }
    if (Serial.read() == '\n') {
      break;
    }
  }
}

/*
void river_crossing() {                                         // this code not required as EDF fans are giving nice output

  //***************************river****************************
  int s = digitalRead(swt);
  if (debug == 1) {
    Serial.print("switch:");
    Serial.println(s);
  }
  if (s == 1) {
    while (1)
    {
      calculate();
      if (debug == 1) {
        Serial.print("Distance front =");
        Serial.println(distance_front);
      }
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
        while (1) {
          calculate();
          valset();
          if (set == 1) {
            break;
          }
        }
        count++;
        led();
        // sett = 3;
      }
      if (debug == 1) {
        Serial.print("Count:");
        Serial.println(count);
      }
      if (count == 2) {
        led();
        turn();
        count = 0;
        break;
      }
    }
  }
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
  //  if (t == 0) {
  for (int i = angle; i >= 60; i--) {
    myservo.write(i);
  }
  delay(350);
  myservo.write(angle);
  delay(700);
  //    t = 1;
  //  }
}
*/

void ledoff() {
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
}

