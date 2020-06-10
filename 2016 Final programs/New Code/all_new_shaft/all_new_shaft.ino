#include <PID_v1.h>                                                     // include library for PID
#include <avr/wdt.h>                                                    // include library for Watchdog

double Setpoint, Input, Output;                                         // variables for PID
double Kp = 12, Ki = 0, Kd = 0.1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);              // declare PID object

const int motorPin = 4;                                                 // declared the pins
const int motorPin1 = 2;
const int pwmPin = 3;
const int trigPin = 11;
const int echoPin = 10;
const int shaft_distance = 35;//29;//12;
const int up = 5;
const int down = 6;

bool up1, down1;                                                        // declare the required variables
int duration, distance;
int pwm;
bool s, debug = 1;

const int led1 = 12,led2  = 13;

void setup() {
  if (debug == 1) {
    Serial.begin (9600);                                                          // serial begin set here
  }
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  myPID.SetOutputLimits(-100, 100); //-50, 50);
  myPID.SetMode(AUTOMATIC);

  Input = shaft_distance;                                                         // Iniatialize variables for the pid
  Setpoint = shaft_distance;

  pinMode(up, INPUT);
  pinMode(down, INPUT);

  up1 = down1 = 0;
  wdt_enable(WDTO_1S);                                                            // watchdog initialized
  s = 0;

  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
}

void measure(void) {
  up1 = digitalRead(up);                                                         // read the pin to see if Joystick button was pressed
  down1 = digitalRead(down);
  if (debug == 1) {
    Serial.print("up");
    Serial.println(up1);
    Serial.print("down");
    Serial.println(down1);
  }

  digitalWrite (trigPin, LOW);
  delayMicroseconds (2);
  digitalWrite (trigPin, HIGH);
  delayMicroseconds (10);
  digitalWrite (trigPin, LOW);

  duration = pulseIn (echoPin, HIGH);                                            // get the value of ultrasonic in cm
  distance = (duration / 2) / 29.1;

  if (debug == 1) {
    Serial.println(distance);
  }
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  digitalWrite(led1,LOW);
  digitalWrite(led2,LOW);
}

void loop() {
  wdt_reset();                                                        // watchdog resets arduino if this function is not called
  measure();                                                          // get the distance of ultrasonic
  Input = distance;                                                   // give the distance to PID
  myPID.Compute();                                                    // PID calculates the Output

  if (debug == 1) {
    Serial.print("output=");
    Serial.println(Output);
  }
/*  
  if(distance == 0){
    digitalWrite(led1,HIGH);
  }
  if(distance >= 70){
    digitalWrite(led2,HIGH);
  }
  if(distance < 70){
    digitalWrite(led2,LOW);
  }
*/
  if (up1 == 0 && down1 == 0) {                                       // if no Joystick button is pressed
    if ((distance >= (shaft_distance - 6)) && (distance <= (shaft_distance + 6))) {              // shaft is adjusted only if distance is in the range
      if (distance < (shaft_distance - 1)) {                                          // if distance is less shaft goes up
        digitalWrite (motorPin, HIGH);
        digitalWrite (motorPin1, LOW);
        pwm = (200 + Output);//(60 + Output);                               // 200+ becuaz new motor requires full power to pull up
        if (pwm < 60) {                                                             // This code because sometimes ultrasonic and
          pwm = 255;                                                          // thus so then PID so to save from Violation
        }
        if (pwm > 255) {                                                        // pwm may go above 255
          pwm = 255;
        }
        analogWrite (pwmPin, pwm);
        if (debug == 1) {
          Serial.print("up pwm=");
          Serial.println(pwm);
        }
        s = 1;
      }
      else if (distance > (shaft_distance + 1)) {                                 // if distance is greater shaft go down
        digitalWrite (motorPin, LOW);
        digitalWrite (motorPin1, HIGH);
        pwm = (20 - Output);
        /*
         very low pwm is given because while going down gravity helps shaft and thus
         if pwm of 60 or more is given then it becomes very difficult to maintain the shaft
         */
        analogWrite (pwmPin, 80);//40                                             // Directly given pwm as PID output gives high value of pwm
        if (debug == 1) {
          Serial.print("Down pwm=");
          Serial.println(pwm);
        }
        s = 0;
      }
      else {                                                                // This code was added to hard stop motor while going down and not when going up
        if (s == 0) {
          digitalWrite (motorPin, LOW);
          digitalWrite (motorPin1, LOW);
          analogWrite (pwmPin, 0);
        } else if (s == 1) {
          digitalWrite (motorPin, LOW);
          digitalWrite (motorPin1, LOW);
          analogWrite (pwmPin, 0);
        }
      }
    }
    else                                                                    // Hard stop the motor if value is not in the range
    { digitalWrite (motorPin, HIGH);
      digitalWrite (motorPin1, HIGH);
      analogWrite (pwmPin, 0);

      digitalWrite(led1,LOW);
    }
  }
  else if (up1 == 1 && down1 == 0) {                                         // if Joystick up is pressed
    digitalWrite (motorPin, HIGH);
    digitalWrite (motorPin1, LOW);
    analogWrite (pwmPin, 255);//120
    if (debug == 1) {
      Serial.print("UP=");
      Serial.println(pwm);
    }
  }
  else if (up1 == 0 && down1 == 1) {                                        // if joystick down is pressed
    digitalWrite (motorPin, LOW);
    digitalWrite (motorPin1, HIGH);
    analogWrite (pwmPin, 200);//60
    if (distance < (shaft_distance + 6))                                     // stop the shaft from going down
    { digitalWrite (motorPin, HIGH);
      digitalWrite (motorPin1, HIGH);
      analogWrite (pwmPin, 0);
    }
    if (debug == 1) {
      Serial.print("DOWN=");
      Serial.println(pwm);
    }
  }
}


/*......................................................Adaptive Tuning...............................................*/
/*
#include <PID_v1.h>
#include <avr/wdt.h>

double Setpoint, Input, Output;
double Kp = 15, Ki = 0, Kd = 0;
double aggKp = 15, aggKi = 0, aggKd = 0;
double consKp = 15, consKi = 0, consKd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int motorPin = 4;                                                 // declared the pins
const int motorPin1 = 2;
const int pwmPin = 3;
const int trigPin = 11;
const int echoPin = 10;
const int shaft_distance = 29;//12;
const int up = A2;
const int down = A3;

bool up1, down1;                                                        // declare the required variables
int duration, distance;
int pwm;
bool s, debug = 1;

void setup() {
  if (debug == 1) {
    Serial.begin (9600);
  }
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  myPID.SetOutputLimits(-100, 100); //-50, 50);
  myPID.SetMode(AUTOMATIC);

  Input = shaft_distance;                                                         // Iniatialize variables for the pid
  Setpoint = shaft_distance;

  pinMode(up, INPUT);
  pinMode(down, INPUT);

  up1 = down1 = 0;
  wdt_enable(WDTO_1S);                                                            // watchdog initialized
  s = 0;
}

void measure(void) {
  up1 = digitalRead(up);                                                         // read the pin to see if Joystick button was pressed
  down1 = digitalRead(down);
  if (debug == 1) {
    Serial.print("up");
    Serial.println(up1);
    Serial.print("down");
    Serial.println(down1);
  }

  digitalWrite (trigPin, LOW);
  delayMicroseconds (2);
  digitalWrite (trigPin, HIGH);
  delayMicroseconds (10);
  digitalWrite (trigPin, LOW);

  duration = pulseIn (echoPin, HIGH);                                            // get the value of ultrasonic in cm
  distance = (duration / 2) / 29.1;

  if (debug == 1) {
    Serial.println(distance);
  }
}

void loop() {
  wdt_reset();                                                        // watchdog resets arduino if this function is not called
  measure();                                                          // get the distance of ultrasonic
  Input = distance;                                                   // give the distance to PID
  myPID.Compute();                                                    // PID calculates the Output

  if (debug == 1) {
    Serial.print("output=");
    Serial.println(Output);
  }

  if (up1 == 0 && down1 == 0) {                                       // if no Joystick button is pressed

    if (distance > shaft_distance)
    { //we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
      //we're far from setpoint, use aggressive tuning parameters
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }

    if ((distance >= (shaft_distance - 6)) && (distance <= (shaft_distance + 6))) {              // shaft is adjusted only if distance is in the range
      if (distance < (shaft_distance - 1)) {                                          // if distance is less shaft goes up
        digitalWrite (motorPin, HIGH);
        digitalWrite (motorPin1, LOW);
        pwm = (200 + Output);//(60 + Output);                               // 200+ becuaz new motor requires full power to pull up
        if (pwm < 60) {                                                             // This code because sometimes ultrasonic and
          pwm = 180;                                                          // thus so then PID so to save from Violation
        }
        if (pwm > 255) {                                                        // pwm may go above 255
          pwm = 255;
        }
        analogWrite (pwmPin, pwm);
        if (debug == 1) {
          Serial.print("up pwm=");
          Serial.println(pwm);
        }
        s = 1;
      }
      else if (distance > (shaft_distance + 1)) {                                 // if distance is greater shaft go down
        digitalWrite (motorPin, LOW);
        digitalWrite (motorPin1, HIGH);
        pwm = (20 - Output);
        /*
         very low pwm is given because while going down gravity helps shaft and thus
         if pwm of 60 or more is given then it becomes very difficult to maintain the shaft
         *
        analogWrite (pwmPin, 80);//40                                             // Directly given pwm as PID output gives high value of pwm
        if (debug == 1) {
          Serial.print("Down pwm=");
          Serial.println(pwm);
        }
        s = 0;
      }
      else {                                                                // This code was added to hard stop motor while going down and not when going up
        if (s == 0) {
          digitalWrite (motorPin, LOW);
          digitalWrite (motorPin1, LOW);
          analogWrite (pwmPin, 0);
        } else if (s == 1) {
          digitalWrite (motorPin, LOW);
          digitalWrite (motorPin1, LOW);
          analogWrite (pwmPin, 0);
        }
      }
    }
    else                                                                    // Hard stop the motor if value is not in the range
    { digitalWrite (motorPin, HIGH);
      digitalWrite (motorPin1, HIGH);
      analogWrite (pwmPin, 0);
    }
  }
  else if (up1 == 1 && down1 == 0) {                                         // if Joystick up is pressed
    digitalWrite (motorPin, HIGH);
    digitalWrite (motorPin1, LOW);
    analogWrite (pwmPin, 255);//120
    if (debug == 1) {
      Serial.print("UP=");
      Serial.println(pwm);
    }
  }
  else if (up1 == 0 && down1 == 1) {                                        // if joystick down is pressed
    digitalWrite (motorPin, LOW);
    digitalWrite (motorPin1, HIGH);
    analogWrite (pwmPin, 150);//60
    if (distance < (shaft_distance + 6))                                     // stop the shaft from going down
    { digitalWrite (motorPin, HIGH);
      digitalWrite (motorPin1, HIGH);
      analogWrite (pwmPin, 0);
    }
    if (debug == 1) {
      Serial.print("DOWN=");
      Serial.println(pwm);
    }
  }
}
*/
/*.........................................................................................................................................*/


/*..........................Problems with Shaft .............................
 * 1. Ultrasonic connection loose or is placed at wrong distance from arena
 * 2. Pulley problem
 * 3. Bearing gets hanged if Tension is not given properly
 *...........................................................................*/
