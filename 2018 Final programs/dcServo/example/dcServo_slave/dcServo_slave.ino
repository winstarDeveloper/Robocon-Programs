#include <Wire.h>
#include <StopWatch.h>
StopWatch MySW(StopWatch::MICROS);
StopWatch tymer(StopWatch::MILLIS);
#include <PID_v1.h>
double Setpoint, Input, Output;
double Kp = 0.05, Ki = 1, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


int in1 = 10;
int in2 = 12;
int pwm = 11;
int interruptPin = 2;
int max_speed = 0;
int desired_speed = 0;
int damping = 0;
int motor_dir = 0;
int encoder_increment = 0;
int mode = 2;
int counter = 0;
double tym = 1;
double rpm, rps;
int position_count = 0;
boolean debug = true;
int debounce_time = 10;
boolean state = 0;
int temp=0;


void incrementCounter() {


  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 'debounce_time', assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > debounce_time )
  { if ( mode == 1)
    {
      counter ++;
      if (debug)
      {
        Serial.print("Count = ");
        Serial.println(counter);
      }

      detachInterrupt(digitalPinToInterrupt(interruptPin));
      if (counter == 1)
        MySW.start();
      if (counter >= 2) {
        tym = MySW.elapsed();
        rpm = (60000000 / tym) ;
        if (debug)
        {
          Serial.print("\t RPM : ");
          Serial.println(rpm);
        }
        counter = 0;
        MySW.reset();
      }
      attachInterrupt(digitalPinToInterrupt(interruptPin), incrementCounter, RISING);
    }
    if (mode == 0)
    { position_count++;

    }
  }
  last_interrupt_time = interrupt_time;
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event

  Serial.begin(9600);           // start serial for output
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(pwm, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), incrementCounter, RISING);

  Input = 0;
  Setpoint = desired_speed;
  myPID.SetMode(AUTOMATIC);
  myPID.SetControllerDirection(DIRECT);
  myPID.SetOutputLimits(40, 255);

}

void loop() {

  Setpoint = desired_speed;
  if (debug)
  {
    Serial.println("inloop");
    //delay(100);
    Serial.print("output");
    Serial.println(Output);
  }

  if (motor_dir == 1 && mode == 1)
  { digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    Input = rpm;
    myPID.Compute();
    analogWrite(pwm, Output);
    if (debug)
    { Serial.print("Output = ");
      Serial.println(Output);
      Serial.println("forwared \t motor_dir == 1 && mode == 1");
    }
  }
  else if (motor_dir == 2 && mode == 1)
  { digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    Input = rpm;
    myPID.Compute();
    analogWrite(pwm, Output);
    if (debug)
    { Serial.print("Output = ");
      Serial.println(Output);
      Serial.println("reverse \t motor_dir == 2 && mode == 1");
    }
  }
  else if (motor_dir == 0)
  { if (debug)
    { Serial.println("**************damping********************");
    }
    for (int i = Output; i > 40 && damping > 0; i--)
    {
      analogWrite(pwm, i);
      delay(damping);
      if (debug) {
        Serial.print("PWM = ");
        Serial.println(i);
      }
    }
    if (debug) {
      Serial.print("****************MOTOR_STOP*****************");

    }

    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    Output = 0;
  }
  else if (mode == 0)
  {
    analogWrite(pwm,desired_speed);
    if (motor_dir == 1)
    { digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
    if (motor_dir == 2)
    { digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    if (position_count == encoder_increment)
    { digitalWrite(in1, HIGH);
      digitalWrite(in2, HIGH);
      motor_dir = 0;
      mode = 1;
      if (debug)
      {
        Serial.println("position reached");
      }
    }
  }


}




void receiveEvent(int howMany)
{ Serial.print("howMany");
  Serial.println(howMany);
  while (Wire.available()) {
    int x = Wire.read();
    if (debug) {
      Serial.print("command = ");
      Serial.println(x);
    }
    if (x == 0)
    { max_speed = Wire.read();
      if (debug)
      { Serial.print("max_speed=");
        Serial.println(max_speed);
      }

    }
    else if (x == 1)
    { desired_speed = Wire.read();
      mode = 1;
      if (debug)
      { Serial.print("desired_speed");
        Serial.println(desired_speed);
      }
      if (desired_speed > max_speed)
      {
        desired_speed = max_speed;
      }
    }
    else if (x == 2) {
      damping = Wire.read();
      damping = map(damping, 0, 255, 0, 50);
      if (debug)
      { Serial.print("damping= ");
        Serial.println(damping);
      }
    }
    else if (x == 3) {
      encoder_increment = Wire.read();
      position_count = 0;
      mode = 0;
      if (debug)
      { Serial.println(" position count ");
        Serial.print(encoder_increment);
      }
    }
    else if (x == 4)
    {
      motor_dir = Wire.read();
      if (debug)
      {
        Serial.print("motor_dir= ");
        Serial.println(motor_dir);
      }

    }
    

    else
    {
      Wire.flush();
    }
  }
}


