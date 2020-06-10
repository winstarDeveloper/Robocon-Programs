

#include <PID_v1.h>
#include <Servo.h>
#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
Servo myservo;  // create servo object to control a se-rvo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

//Specify the links and initial tuning parameters
//double Kp=10, Ki=0, Kd=0;
/****for rraspberry pi***/
double Kp = 0.95, Ki = 0, Kd = 0.05;
//double Kp = 0.50, Ki = 0, Kd = 0.05;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
char a;
double b = 0;
int c = 0;
int btn=3;

int last_pos = 90;

void setup()
{
  //initialize the variables we're linked to
 pinMode(10,OUTPUT);
 Serial.begin(9600);
  myservo.attach(A0);  // attaches the servo on pin 9 to the servo object
  

pinMode(3,INPUT);
  Input = 160;
  Setpoint = 160;
 

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-45, 45);
    for (int i = 0; i <= 90; i++)
    {
      last_pos = i;
      myservo.write(i);
      // Serial.println(i);
      delay(15);
    }
     
  }

int temp;
  void loop()
  {
   // Setpoint = 1.6;
   if(digitalRead(btn)==HIGH)
   {
     Serial.write('c');
   }
    while (Serial.available()>0) {

      b = Serial.parseInt();
      if(b==999)
      {
      myservo.write(150);
      delay(10);
     
      //delay(50);
      }
      else if(b==998)
      {
       myservo.write(45);
      delay(15);
     
       //delay(50);
      }
      else if(b==997)
      {
        myservo.write(135);
      delay(15);
     
        //delay(50);
      }
      else
      {
      /*   Serial.print("b :");
         Serial.println(b);
         */
      //delay(500);
      //b=Serial.read();
      Input = b;
   //   Serial.println(b);
      // Serial.print("Input");
      //Serial.println(Input);
      myPID.Compute();
      //Serial.print("Output");
      //Serial.println(Output);
      b = 0;
      
      temp = 90 - Output;
//Serial.println(temp);;
      myservo.write(temp);
      delay(15);
      
      }
      //Serial.print("temp");
      //Serial.println(temp);
      /*if(Input<3.2)
       {
         if(last_pos<(90+Output))
         {
         for(int i=last_pos;i<(90+Output);i++)
         {
            myservo.write(i);
            last_pos=i;
              delay(15);
         }
         Serial.print("Left : last pos");
         Serial.println(last_pos);
         }
         else
         {
            for(int i=last_pos;i<(90+Output);i--)
         {
            myservo.write(i);
            last_pos=i;
              delay(15);
         }
           Serial.print("Left : last pos");
         Serial.println(last_pos);
         }
       }

       else
       {
         if(last_pos<(90+Output))
         {
          for(int i=last_pos;i>(90+Output);i++)
         {
            myservo.write(i);

            delay(15);
         }
             Serial.print("Right : last pos");
         Serial.println(last_pos);
         }
         else
         {
           for(int i=last_pos;i>(90+Output);i--)
         {
            myservo.write(i);

            delay(15);
         }
             Serial.print("Right : last pos");
         Serial.println(last_pos);
         }
       }

      /*
       else if(Input<3.2)
       {
          if(Output>last_pos)
       {
         for(int i=last_pos;i<(90+Output);i++)
         {
            myservo.write(i);

            delay(15);
         }
       }

       else
       {
          for(int i=last_pos;i>(90+Output);i--)
         {
            myservo.write(i);
            delay(15);
         }

       }
         }





         else{
           }
           
         */

      if (Serial.read() == '\n') {
        break;
      }

    }

  }


