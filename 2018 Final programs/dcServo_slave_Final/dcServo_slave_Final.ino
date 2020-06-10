// correct RPM
// set debounce time

#define DEBUG 1

#include <avr/wdt.h>

#include <Wire.h>

#include <StopWatch.h>
StopWatch MySW(StopWatch::MICROS);

#include <PID_v1.h>
double Setpoint, Input, Output;
double Kp = 0.02, Ki = 2.9, Kd = 0;
double gotKp = 0, gotKi = 0, gotKd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

#define ADDRESS 40

int in1 = 5;
int in2 = 7;
int pwm = 6;

int interruptPin = 2, channelB = 2;

int runSpeed = 0;
double runPWM = 0;
int dampConst = 0, damping = 0;
int motorDir = 0, posCount = 0;
int count = 0;
int command = -1;
int lineEndCount = 0, endCount = 0, speeed = 0, lineCount = 0;

unsigned long startTym;

boolean runMotor = false, stat = false, runStat = false;
boolean startCOUNT = false, countProblem = false;

boolean gotIntr = false;

double tym = 1, prev_tym;
double rpm, rps;

#define DAMP_DELAY 500
int position_count = 0;
int debounce_time = 1;
boolean state = 0;
int temp = 0;

static unsigned long last_interrupt_time = 0;
unsigned long interrupt_time = millis(), start, End;

void incrementCounter() {
  position_count ++;
  //  Serial.print("Count : ");
  //  Serial.println(position_count);
  if (position_count == 1)
    MySW.start();
  if (position_count == 2) {
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    tym = MySW.elapsed();
    //if (tym < ) {
    double x = tym * floor(135 / 16);//34;;
    rpm = (60000000 / x);
    if (DEBUG) {
      //  Serial.print("\t TIME : ");
      //  Serial.print(tym);
      //  Serial.print("\t RPM : ");
      //  Serial.print(rpm);
    }
    //}
    position_count = 0;
    //prev_tym = tym;
    MySW.reset();
    //    MySW.slowStop();
    attachInterrupt(digitalPinToInterrupt(interruptPin), incrementCounter, RISING);
  }
  if (position_count > 2) {
    position_count = 0;
  }
}

void incCount() {
  digitalWrite(A2, HIGH);
  interrupt_time = millis();
  // If interrupts come faster than 'debounce_time', assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > debounce_time ) {
    count++;
    if (DEBUG) {
      Serial.print("Count : ");
      Serial.println(count);
    }
  }
  last_interrupt_time = interrupt_time;
  if (count >= posCount) {
    hardStop();
    End = millis();
    if (DEBUG) {
      Serial.print("Time : "); Serial.println(End - start);
    }
    count = 0;
  }
}

void incLineCount() {
  digitalWrite(A2, HIGH);
  interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > debounce_time ) {
    lineCount++;
    if (DEBUG) {
      Serial.print("Line Count : ");
      Serial.println(lineCount);
    }
  }
  last_interrupt_time = interrupt_time;
}

void accelerate() {
  for (int i = 0; i < runPWM; i++) {
    analogWrite(pwm, i);
    delayMicroseconds(1900);
  }
}

void setup() {
  Wire.begin(ADDRESS);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event

  wdt_enable(WDTO_4S);

  Serial.begin(9600);           // start serial for output

  pinMode(interruptPin, OUTPUT);
  digitalWrite(interruptPin, HIGH);
  delay(100);
  digitalWrite(interruptPin, LOW);
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  Input = 0;

  myPID.SetMode(AUTOMATIC);
  myPID.SetControllerDirection(DIRECT);
  myPID.SetOutputLimits(20, 255);

  Serial.println("DC Servo Started");
}

#define SAMPLES 1
long getFrequency(int pin) {
  long freq = 0;
  for (unsigned int j = 0; j < SAMPLES; j++)
    freq += 500000 / pulseIn(pin, HIGH);
  return freq / SAMPLES;
}

void pidPWM() {
  if (DEBUG) {
    if (command == 1) {
      //  Serial.println(" GOING FORWARD ");
    } else if (command == 2) {
      //  Serial.println(" GOING REVERSE ");
    }
  }
  //  long rpm = getFrequency(2);
  //  rpm = map(rpm, 632, 1024, 291, 468);
  Setpoint = runSpeed;
  Input = (double)rpm;
  myPID.Compute();
  runPWM = Output;
  if (runStat) {
    //accelerate();
    runStat = false;
  }
  analogWrite(pwm, runPWM);
  if (DEBUG) {
    Serial.print("\t TIME : ");
    Serial.print(tym);
    Serial.print(" RPM : ");
    Serial.print(rpm);
    Serial.print("\tOutput : ");
    Serial.print(Output);
    Serial.print("\trunPWM : ");
    Serial.println(runPWM);
    //Serial.println("");
  }
}

void slowStop() {
  digitalWrite(A0, LOW);                  // led for indication
  digitalWrite(A1, LOW);                  // led for indication
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  detachInterrupt(digitalPinToInterrupt(channelB));

  if (damping == 1) {
    hardStop();
    analogWrite(pwm, 0);
  } else {
    damping = (damping / 100) * 255;
    if (damping > runPWM) {
      damping = runPWM;
    }

    if (DEBUG) {
      Serial.print(" Resultant Damping : ");
      Serial.println(damping);
    }
    analogWrite(pwm, damping);
    if (runPWM > 0)
      for (int i = damping; i > 0; i--) {
        analogWrite(pwm, i);
        delayMicroseconds(1900);
        if (DEBUG) {
          Serial.print(" PWM : ");
          Serial.println(i);
        }
      }
    analogWrite(pwm, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    if (DEBUG) {
      Serial.print(" slow STOPPING MOTOR ");
    }
  }
  countProblem = false;
  lineCount = 0;
  lineEndCount = 0;
  endCount = 0;
  speeed = 0;
}

void hardStop() {
  digitalWrite(A0, LOW);                  // led for indication
  digitalWrite(A1, LOW);                  // led for indication

  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  //delay(100);
  //analogWrite(pwm, 0);
  if (DEBUG) {
    Serial.print(" HARD Stop MOTOR ");
  }
  detachInterrupt(digitalPinToInterrupt(channelB));
}

void getOnLine() {
  if (lineCount > lineEndCount && lineCount <= endCount) {
    analogWrite(pwm, speeed);
    if (lineCount >= endCount) {
      hardStop();
      if (!(speeed >= 30 && speeed <= 50)) {
        analogWrite(pwm, 0);
      }
      detachInterrupt(digitalPinToInterrupt(channelB));
      lineEndCount = 0;
      endCount = 0;
      speeed = 0;
      countProblem = true;
    }
    if (DEBUG) {
      Serial.print("Slow Speed ongoing : ");
      Serial.println(speeed);
    }
  }
}

void loop() {
  wdt_reset();
  if (runMotor && (command == 1 || command == 2)) {
    pidPWM();
  }
}

void receiveEvent(int howMany) {
  if (DEBUG) {
    //Serial.print("\n\nhowMany : ");
    //Serial.println(howMany);
  }
  command = -1;
  //while (Wire.available()) {
  if (Wire.available()) {
    command = Wire.read();
    if (command == 0) {
      damping = Wire.read();
      runMotor = false;
      runStat = false;
      slowStop();
      if (DEBUG) {
        Serial.println("Got command 0 slowStop MOTOR");
        Serial.print(" Command : "); Serial.print(command);
        Serial.print(" Damping : "); Serial.println(damping);
      }
    } else if (command == 1) {
      runSpeed = Wire.read();
      runSpeed = runSpeed + Wire.read();
      runSpeed = runSpeed + 10;                         // remove when done
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      runMotor = true;
      runStat = true;
      digitalWrite(A0, HIGH);           // led for indication
      attachInterrupt(digitalPinToInterrupt(interruptPin), incrementCounter, RISING);
      if (DEBUG) {
        Serial.print("Got command 1 FORWARD");
        Serial.print(" Command : "); Serial.print(command);
        Serial.print("\t runSpeed : "); Serial.println(runSpeed);
      }
    } else if (command == 2) {
      runSpeed = Wire.read();
      runSpeed = runSpeed + Wire.read();
      runSpeed = runSpeed + 10;                            // remove when done
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      runMotor = true;
      runStat = true;
      digitalWrite(A1, HIGH);           // led for indication
      attachInterrupt(digitalPinToInterrupt(interruptPin), incrementCounter, RISING);
      if (DEBUG) {
        Serial.print("Got command 2 REVERSE");
        Serial.print(" Command : "); Serial.print(command);
        Serial.print("\t runSpeed : "); Serial.println(runSpeed);
      }
    } else if (command == 3) {
      if (DEBUG) {
        Serial.println("Got command 3 GO TO POSITION");
      }
      motorDir = Wire.read();
      runSpeed = Wire.read();
      posCount = 0;

      posCount = posCount + Wire.read();
      posCount = posCount + Wire.read();
      posCount = posCount + Wire.read();
      /*
        while (Wire.available()) {
        posCount = posCount + Wire.read();;
        }
      */
      if (motorDir) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(A0, HIGH);
      } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(A1, HIGH);
      }
      analogWrite(pwm, runSpeed);
      position_count = 0;
      //debounce_time = ((0.01666666667 * runSpeed) * 135);
      //      if (DEBUG) {
      //        Serial.print(" Debounce : ");
      //        Serial.println(debounce_time);
      //      }
      start = millis();
      attachInterrupt(digitalPinToInterrupt(channelB), incCount, RISING);
      if (DEBUG) {
        Serial.println("Got command 3 GO TO POSITION");
        Serial.print(" Command : "); Serial.print(command);
        Serial.print(" motorDir : "); Serial.print(motorDir);
        Serial.print(" runSpeed : "); Serial.print(runSpeed);
        Serial.print(" posCount : "); Serial.println(posCount);
      }
    } else if (command == 4) {
      /*
        int temp = 0;
        boolean stat = false, stat2 = false;
        while (Wire.available()) {
        while (Wire.available() && !stat2) {
          while (Wire.available() && !stat) {
            temp = Wire.read();
            if (temp == 0) {
              stat = true;
              break;
            }
            gotKp = gotKp + temp;
          }
          temp = Wire.read();
          if (temp == 0) {
            stat2 = true;
            break;
          }
          gotKi = gotKi + temp;
        }
        temp = Wire.read();
        gotKd = gotKd + temp;
        }
        gotKp = gotKp / 1000;
        gotKi = gotKi / 1000;
        gotKd = gotKd / 1000;
        myPID.SetTunings(gotKp, gotKi, gotKd);
        if (DEBUG) {
        Serial.print(" PID values " );
        Serial.print(" :- Kp : ");
        Serial.print(gotKp);
        Serial.print(" Ki : ");
        Serial.print(gotKi);
        Serial.print(" Kd : ");
        Serial.println(gotKd);
        }*/
    } else if (command == 5) {
      runPWM = Wire.read();
      if (!countProblem) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(A0, HIGH);
      }
      if (lineCount <= lineEndCount)
        analogWrite(pwm, runPWM);
      getOnLine();
      if (DEBUG) {
        Serial.print("Got command 5 Forward PWM");
        Serial.print("\tGot PWM : ");
        Serial.println(runPWM);
      }
    } else if (command == 6) {
      runPWM = Wire.read();
      if (!countProblem) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(A1, HIGH);
      }
      if (lineCount <= lineEndCount)
        analogWrite(pwm, runPWM);
      getOnLine();
      if (DEBUG) {
        Serial.print("Got command 6 Reverse PWM");
        Serial.print("\tGot PWM : ");
        Serial.println(runPWM);
      }
    } else if (command == 7) {
      myPID.SetTunings(Kp, Ki, Kd);
      if (DEBUG) {
        Serial.println("PID set to default");
      }
    } else if (command == 8) {
      lineEndCount = Wire.read();
      endCount = Wire.read();
      speeed = Wire.read();
      lineCount = 0;
      countProblem = false;
      if (DEBUG) {
        Serial.print("Line-End Count : ");
        Serial.print(lineEndCount);
        Serial.print("End Count : ");
        Serial.print(endCount);
        Serial.print("Speed : ");
        Serial.print(speeed);
      }
      attachInterrupt(digitalPinToInterrupt(channelB), incLineCount, RISING);
    } else {
      runMotor = false;
      slowStop();
      if (DEBUG) {
        Serial.print("Got unknown command : ");
        Serial.println(command);
      }
    }
    Wire.flush();
    /*
      startTym = millis();
      while (Wire.available()) {
      Wire.read();
      //Wire.flush();
      if(startTym+1000 > millis()){
        break;
        Wire.flush();
      }
      }*/
  }
}
