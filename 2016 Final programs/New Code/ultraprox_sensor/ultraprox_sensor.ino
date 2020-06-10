#include <PID_v1.h>
#include <Servo.h>
Servo myservo;

const int trig = 10, echo = 9;
int set, sett, distance_front, count;
long duration_front;
const int led1 = 11, led2 = 12, led3 = 0;
bool t = 0;
const int angle = 93;

double Setpoint, Input, Output;
double Kp = 1 , Ki = 0, Kd = 0;//0.125 0.1;
//double Kp = 0.7 , Ki = 0, Kd = 0.1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int sensor = A1;//A0;
int swt = 3;//5;
bool stat;
const int junctionPulse = A2;//13;

bool s1 = 0;
int add = 0;

bool debug = 0;
int temp = 0, angle1, s;
int tt = 0;

void setup() {
  if (debug == 1) {
    Serial.begin(9600);
  }
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(13, OUTPUT);
  set = sett = count = 0;                                                         // initailize the variables

  pinMode(led1 , OUTPUT);
  pinMode(led2 , OUTPUT);
  pinMode(led3 , OUTPUT);

  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);
  digitalWrite(led3, HIGH);

  myservo.attach(A0);//10);
  t = 0;

  pinMode(sensor, INPUT);
  pinMode(swt, INPUT);
  pinMode(12, OUTPUT);
  pinMode(junctionPulse, INPUT);

  Input = 33;
  Setpoint = 33;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-35, 35);

  for (int i = 0; i <= angle; i++)
  {
    myservo.write(i);
    delay(15);
  }

  myservo.write(angle);

  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);

  s1 = add = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  ledoff();                                                                     // turning the led off
  if (debug == 1) {
    Serial.print("switch:");
    Serial.println(s);
  }
  int temp;
  temp = analogRead(sensor);                                                // taking the sensor value
  temp = map(temp, 0, 1023, 0, 255);                                            // map the sensor output from 1023 to 255
  temp = ((temp * 0.02) / 4.5) * 70;                                            // calculating the value of temp accordinng to the manual
  if (debug == 1) {
    Serial.print("Original:");
    Serial.println(temp);
  }

  if (temp >= 65) {                                                              // keeping the castor turned if line missed // SEE HERE
    if (angle1 > 90) {
      temp = 65;
      //      tt = 1;                                                                   // tt varaible used for giving direct value to servo
    }
    else if (angle1 < 90) {
      temp = 0;
      //      tt = 2;
    }
  }

  if (angle1 >= 80 && angle1 <= 100 && (temp == 65 || temp == 0)) {                                 // code to keep the castor straight if value misses
    temp = 35;
    while (1) {
      myservo.write(angle);
      getSensorValue();
      if (temp >= 0 && temp < 65) {
        break;
      }
    }
  }

  if (debug == 1) {
    Serial.println(temp);
  }

  Input = temp; //ceil (temp);                                              // give the value of the sensor to the PID
  myPID.Compute();                                                          // calculate the pid output
  angle1 = 90 - Output;                                                     // calculate the angle for the servo

  if (temp >= 45) {                                                         // Increasing the Angle for turning in the River only useful when Junction crossed
    angle1 += add;
  }
  if (temp <= 25) {
    angle1 -= add;
  }
  /*
    if (tt == 1) {
      angle1 = 140;
      tt = 0;
    }
    else if (tt = 2) {
      angle1 = 50;
      tt = 0;
    }
  */
  //  if (temp != 20) {                                                           // This condition added becauz sensor giving value 20.. maybe second sensor problem
  myservo.write(angle1);                                                    // turn the castor MAIN
  delay(15);
  //  }

  if (debug == 1) {
    Serial.print("angle:");
    Serial.println(angle1);
  }

  stat = digitalRead(junctionPulse);                                        // read the Junction pulse

  if (debug == 1) {
    Serial.print("Junction:");
    Serial.println(stat);
  }

  if (stat == HIGH && s == 0) {                                             // Turn the Castor if the Junction is received
    if (debug == 1) {
      Serial.println("Junction Turn");
    }
    if (s1 == 0) {
      angle1 = 145;//135;
      myservo.write(angle1);
      delay(1000);
      s1 = 1;                                                              // This code will run only once
      add = 10;                                                            // This value is added to the Angle for better performance on river
    }
  }

  /************************This code for Eco bot straight to Highland using pneumatic*******************************

  s = digitalRead(swt);                                                    // take the value of the Button
  s = 0;
  if (s == 1) {                                                            // check if the button was pressed
    while (1)                                                              // Enter into loop
    {
      calculate();                                                         // get the valur of ultrasonic
      if (debug == 1) {
        Serial.print("Distance front =");
        Serial.println(distance_front);
      }
      myservo.write(angle);                                                // keep the castor straight
      valset();                                                            // sees if the value is in the range
      if (set == 1) {                                                      // first island is detected
        sett = 1;
      }
      if (sett == 1 && set == 0) {                                         /// first island ends
        sett = 2;
        while (1) {
          calculate();
          if (distance_front > 40) {
            break;
          }
        }
      }
      valset();                                                            // sees if island is in the range
      if (sett == 2 && set == 1) {                                         // second island detected
        while (1) {
          calculate();
          valset();
          if (set == 1) {                                                   // Loop breaks if Out of Range
            break;
          }
        }
        count++;                                                          // increase the counter
        led();                                                            // turns the led On
      }

      if (debug == 1) {
        Serial.print("Count:");
        Serial.println(count);
      }

      if (count == 2) {                                                   // Second island detected and/or completed
        led();
        turn();                                                           // turn the castor
        count = 0;                                                        // reset the counter variable
        break;                                                            // terminate from the River Code Loop and Sensor is now ON
      }
    }
  }
  */
}

void getSensorValue() {
  temp = analogRead(sensor);                                                // taking the sensor value
  temp = map(temp, 0, 1023, 0, 255);                                            // map the sensor output from 1023 to 255
  temp = ((temp * 0.02) / 4.5) * 70;                                            // calculating the value of temp accordinng to the manual
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

void valset() {                                                   // sees if island is in the range
  if (distance_front > 40) {
    set = 1;
  }
  if (distance_front <= 40) {
    set = 0;
  }
}

void led() {                                                    // code to turn ON led if the island is detected
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

void ledoff() {                                                 // code to turn off the led
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
}

void turn() {                                                   // Turn the castor after the Second island is detected or is completed
  for (int i = angle; i >= 60; i--) {
    myservo.write(i);
  }
  delay(350);                                                   // delay has been adjusted according to some practical tests
  myservo.write(angle);                                         // castor is straight
  delay(700);
}


// check the value of sensor without line
// test if the sensor is really fast or not
