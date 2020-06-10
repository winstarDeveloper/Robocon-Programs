#include "hidjoystickrptparser.h"

JoystickReportParser::JoystickReportParser(JoystickEvents *evt) :
  joyEvents(evt),
  oldHat(0xDE),
  oldButtons(0) {
  for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
    oldPad[i] = 0xD;
}


//for x axis
int pwm;

int val, val2, val_diag1, val_diag2;
int forward_left;
int Rz_button_value;





const int input1 = A10;//A8;
//const int input2=A14;
const int input3 = A11;
const int input4 = A11; /*...............relay for upper bat service ..................*/
const int input2 = A9;
/*******stepper -pins***********/
/*const int in1=A8;
  const int in2=A9;
  const int in3=A10;
  const int in4=A11;*/
const int dir = A9;
const int pull = A8;
int count = 30;
//int buttonstate=0;

const int pwm1 = 9;
const int pwm2 = 8;
const int pwm3 = 6;
const int pwm4 = 5;

/*const  int motor1in1=47;
  const  int motor1in2=49;
  const  int motor2in1=43;
  const  int motor2in2=45;
  const   int motor3in1=39;
  const  int motor3in2=41;
  const   int motor4in1=35;
  const  int motor4in2=37;*/
/*
    const  int motor1in1=49;
   const  int motor1in2=47;
   const  int motor2in1=45;
   const  int motor2in2=43;
   const   int motor3in1=41;
   const  int motor3in2=39;
  const   int motor4in1=37;
   const  int motor4in2=35;*/
const  int motor1in1 = 49;
const  int motor1in2 = 47;
const  int motor2in1 = 45;
const  int motor2in2 = 43;
const   int motor3in1 = 41;
const  int motor3in2 = 39;
const   int motor4in1 = 35;
const  int motor4in2 = 37;
void fullpwm()
{
  analogWrite(pwm1, 255);
  analogWrite(pwm2, 255);
  analogWrite(pwm3, 255);
  analogWrite(pwm4, 255);
}
void norstop(int motornin1, int motornin2)
{
  digitalWrite(motornin1, LOW);
  digitalWrite(motornin2, LOW);
}
void zeropwm()
{
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
  analogWrite(pwm3, 0);
  analogWrite(pwm4, 0);
}

void formot(int motornin1, int motornin2)
{
  digitalWrite(motornin1, HIGH);
  digitalWrite(motornin2, LOW);
}
void revmot(int motornin1, int motornin2)
{
  digitalWrite(motornin1, LOW);
  digitalWrite(motornin2, HIGH);
}
void hardstopmot(int motornin1, int motornin2)
{
  digitalWrite(motornin1, HIGH);
  digitalWrite(motornin2, HIGH);
}
void left()
{
  hardstopmot(motor1in1, motor1in2);

  hardstopmot(motor2in1, motor2in2);

  formot(motor3in1, motor3in2);

  revmot(motor4in1, motor4in2);
}
void right()
{
  hardstopmot(motor1in1, motor1in2);

  hardstopmot(motor2in1, motor2in2);

  revmot(motor3in1, motor3in2);

  formot(motor4in1, motor4in2);
}
void forward()
{

  formot(motor1in1, motor1in2);

  revmot(motor2in1, motor2in2);

  hardstopmot(motor3in1, motor3in2);

  hardstopmot(motor4in1, motor4in2);

}
void reverse()
{
  revmot(motor1in1, motor1in2);

  formot(motor2in1, motor2in2);

  hardstopmot(motor3in1, motor3in2);

  hardstopmot(motor4in1, motor4in2);

}
void diagupleft()
{
  formot(motor1in1, motor1in2);

  revmot(motor2in1, motor2in2);

  formot(motor3in1, motor3in2);

  revmot(motor4in1, motor4in2);
}
void diagupright()
{
  formot(motor1in1, motor1in2);

  revmot(motor2in1, motor2in2);

  revmot(motor3in1, motor3in2);

  formot(motor4in1, motor4in2);

}
void diagdownleft()
{

  revmot(motor1in1, motor1in2);

  formot(motor2in1, motor2in2);

  formot(motor3in1, motor3in2);

  revmot(motor4in1, motor4in2);

}
void diagdownright()
{
  revmot(motor1in1, motor1in2);

  formot(motor2in1, motor2in2);

  revmot(motor3in1, motor3in2);

  formot(motor4in1, motor4in2);

}

void hardstop()
{

  /*  hardstopmot(motor1in1,motor1in2);

    hardstopmot(motor2in1,motor2in2);

     hardstopmot(motor3in1,motor3in2);

      hardstopmot(motor4in1,motor4in2);

      fullpwm();*/
  //delay(100);
  norstop(motor1in1, motor1in2);

  norstop(motor2in1, motor2in2);

  norstop(motor3in1, motor3in2);

  norstop(motor4in1, motor4in2);



  zeropwm();

}



void rotateright()
{
  revmot(motor1in1, motor1in2);

  revmot(motor2in1, motor2in2);

  revmot(motor3in1, motor3in2);

  //  revmot(motor4in1,motor4in2);
  formot(motor4in1, motor4in2);

}

void rotateleft()
{
  formot(motor1in1, motor1in2);

  formot(motor2in1, motor2in2);

  formot(motor3in1, motor3in2);

  // formot(motor4in1,motor4in2);
  revmot(motor4in1, motor4in2);

}
void medpwm()
{
  analogWrite(pwm1, 175);
  analogWrite(pwm2, 175);
  analogWrite(pwm3, 175);
  analogWrite(pwm4, 175);
}
void norpwm()
{
  analogWrite(pwm1, 160);
  analogWrite(pwm2, 160);
  analogWrite(pwm3, 160);
  analogWrite(pwm4, 160);
}

void JoystickReportParser::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  bool match = true;

  // Checking if there are changes in report since the method was last called
  for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
    if (buf[i] != oldPad[i]) {
      match = false;
      break;
    }

  // Calling Game Pad event handler
  if (!match && joyEvents) {
    joyEvents->OnGamePadChanged((const GamePadEventData*)buf);

    for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++) oldPad[i] = buf[i];
  }

  uint8_t hat = (buf[5] & 0xF);

  // Calling Hat Switch event handler
  if (hat != oldHat && joyEvents) {
    joyEvents->OnHatSwitch(hat);
    oldHat = hat;
  }

  uint16_t buttons = (0x0000 | buf[6]);
  buttons <<= 4;
  buttons |= (buf[5] >> 4);
  uint16_t changes = (buttons ^ oldButtons);

  // Calling Button Event Handler for every button changed
  if (changes) {
    for (uint8_t i = 0; i < 0x0C; i++) {
      uint16_t mask = (0x0001 << i);

      if (((mask & changes) > 0) && joyEvents) {
        if ((buttons & mask) > 0)
          joyEvents->OnButtonDn(i + 1);
        else
          joyEvents->OnButtonUp(i + 1);
      }
    }
    oldButtons = buttons;
  }
}

void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt) {
  Serial.print("X1: ");
  PrintHex<uint8_t > (evt->X, 0x80);
  Serial.print("\tY1: ");
  PrintHex<uint8_t > (evt->Y, 0x80);
  Serial.print("\tX2: ");
  PrintHex<uint8_t > (evt->Z1, 0x80);
  Serial.print("\tY2: ");
  PrintHex<uint8_t > (evt->Z2, 0x80);
  Serial.print("\tRz: ");
  PrintHex<uint8_t > (evt->Rz, 0x80);
  Serial.println("");

  val2 = evt->X;
  val = evt->Y;
  delay(10);
  Rz_button_value = evt->Rz;



  //for y axis
  //hard stop
  if ((val > 121 && val2 < 135) && (val2 > 121 && val < 135))
  { /*.............hard stop....initial condition................*/
    //  Serial.println("In 5th if");
    hardstop();

    //    fullpwm();

  }



  if (val < 120)

  {
    Serial.println("In 2st if");
    pwm = val - 129;
    pwm = 2 * pwm;
    //   stop_mot();
    /******  downward movement *******/
    // Serial.print("value of pwm is ");
    // Serial.print(pwm);
    if ((val2 >= 0 && val2 <= 100) && (val >= 150 && val <= 255))
    {
      //Serial.println("Downwardleft movement");

      diagdownleft();

      norpwm();

    }
    else if ((val2 >= 150 && val2 <= 255) && (val >= 150 && val <= 255))
    {
      //   Serial.println("Downward right movement");

      diagdownright();

      norpwm();

    }
    else {
      //motor 1
      /***************DOWNWARD MOVEMENT**********/

      reverse();
      norpwm();

      /*analogWrite(pwm1,pwm); // sets pin 12 HIGH
        analogWrite(pwm2, pwm);
        analogWrite(pwm3, 255);
        analogWrite(pwm4, 255);*/
    }
  }

  //for x axis

  if (val2 < 120)
  {
    //Serial.println("In 3rd if");
    pwm = 127 - val2;
    pwm = 2 * pwm;
    //  stop_mot();
    if ((val2 >= 0 && val2 <= 100) && (val >= 0 && val <= 100))
    {
      // Serial.println("Forward left movement");

      diagupleft();

      norpwm();


    }
    else if ((val2 >= 0 && val2 <= 100) && (val >= 150 && val == 255))
    {
      // Serial.println("Downwardleft movement");
      diagdownleft();

      norpwm();

    }
    else
    {
      //left movement
      // Serial.print("value of pwm is ");
      //  Serial.print(pwm);
      left();
      norpwm();
      /*analogWrite(pwm1,255); // sets pin 12 HIGH
        analogWrite(pwm2,255);
         analogWrite(pwm3,pwm);
           analogWrite(pwm4,pwm); */
    }

  }
  if (val2 > 129)
  {
    Serial.println("In 4th if");
    pwm = val2 - 129;
    pwm = 2 * pwm;
    //    Serial.print("value of pwm is ");
    //     Serial.print(pwm);
    //   stop_mot();

    if ((val2 >= 150 && val2 <= 255) && (val >= 0 && val <= 100))
    {

      //   Serial.println("Upward right movement");

      diagupright();

      norpwm();

    }
    else if ((val2 >= 150 && val2 <= 255) && (val >= 150 && val <= 255))
    {
      Serial.println("Downward right movement");

      diagdownright();

      norpwm();
    }
    else
    {

      /*........RIHJT ...........*/
      //motor 1
      pwm = val2 - 129;
      pwm = 2 * pwm;

      right();
      norpwm();
      /*analogWrite(pwm1, 255); // sets pin 12 HIGH
        analogWrite(pwm2, 255);
        analogWrite(pwm3,pwm);
        analogWrite(pwm4,pwm);
      */
    }


  }
  /*
  if (Rz_button_value == 31) //used for relay
  {
    /*............upper bat service....*

    //Serial.println("Relay operation upper bat reply ");

    /* digitalWrite(input4,LOW);
       delay(200);
      digitalWrite(input4,HIGH);
    *
    digitalWrite(input4, LOW);
    digitalWrite(input3, LOW);
    //digitalWrite(input2,LOW);
    delay(200);
    digitalWrite(input4, HIGH);
    digitalWrite(input3, HIGH);
    //delay(150);
    //digitalWrite(input2,HIGH);

  }
  if (Rz_button_value == 47) //used for relay
  {
    /*.............right bat   reply  ................*

    // Serial.println("Relay operation right bat reply");
    /* digitalWrite(input4,LOW);
      delay(200);
      digitalWrite(input4,HIGH);
    *
  }
  if (Rz_button_value == 143) //used for relay
  {
    /*digitalWrite(input3,LOW);
      delay(200);
      digitalWrite(input3,HIGH);*
  }

  if (Rz_button_value == 79) //used for relay
  {
    /*................service bat reply...................*

    Serial.println("Relay operation");
    digitalWrite(input2, LOW);

    delay(350);
    digitalWrite(input2, HIGH);


  }
*/
}

void JoystickEvents::OnHatSwitch(uint8_t hat) {
  Serial.print("Hat Switch: ");
  PrintHex<uint8_t > (hat, 0x80);
  Serial.println("");
}

void JoystickEvents::OnButtonUp(uint8_t but_id) {
  Serial.print("Up: ");
  Serial.println(but_id, DEC);

  if (but_id == 7)
  {

    //  Serial.println("Stop Round movement");
    // hardstop();
    //fullpwm();
    hardstop();
    //fullpwm();
  }
  if (but_id == 8)
  {

    //  Serial.println("Stop Round movement");
    //hardstop();
    hardstop();    // fullpwm();

  }

}

void JoystickEvents::OnButtonDn(uint8_t but_id) {
  Serial.print("Dn: ");
  Serial.println(but_id, DEC);


  if (but_id == 7)
  {
    //       Serial.println("Right Round Movement");
    rotateright();
    // medpwm();
    analogWrite(pwm1, 100);
    analogWrite(pwm2, 100);
    analogWrite(pwm3, 100);
    analogWrite(pwm4, 100);
  }


  if (but_id == 8)
  {
    //       Serial.println("Left Round movement");
    rotateleft();
    // medpwm();
    analogWrite(pwm1, 100);
    analogWrite(pwm2, 100);
    analogWrite(pwm3, 100);
    analogWrite(pwm4, 100);

  }

  if (but_id == 10)
  {

    digitalWrite(input3, LOW);
    delay(1000);
    digitalWrite(input1, LOW);
   // delay(225);
    delay(200);                       // THIS IS THE DELAY
    digitalWrite(input3, HIGH);
    //delay(100);
    //digitalWrite(input1,HIGH);

    delay(700);
    digitalWrite(input1, HIGH);

    //Serial.println("Right Round Movement");
    //    rotateright();

    //    medpwm();
    //    delay(650);
    //       Serial.println("Stop Round movement");
    //    hardstop();
    //    fullpwm();
  }

  /*if(but_id==5)
    {
     Serial.println("  Stepper reverse operation");
    digitalWrite(dir, LOW);
    for (int i=0;i<count;i++)
    {

      digitalWrite(pull, HIGH);
      delay(10);
      digitalWrite(pull, LOW);
      delay(10);
    }
    }*/

  /*if(but_id==9)
    {  Serial.println("  Stepper operation");
    digitalWrite(dir, HIGH);
    for (int i=0;i<count;i++)
    {

      digitalWrite(pull, HIGH);
      delay(10);
      digitalWrite(pull, LOW);
      delay(10);
    }
    }*/
  if (but_id == 3) //used for relay
  {
    /*............upper bat service....*/

    Serial.println("Relay operation upper bat reply ");

    /* digitalWrite(input4,LOW);
      -  delay(200);
      digitalWrite(input4,HIGH);
    */
    digitalWrite(input4, LOW);
    digitalWrite(input3, LOW);
    //digitalWrite(input2,LOW);
    delay(200);
    digitalWrite(input4, HIGH);
    digitalWrite(input3, HIGH);
    //delay(150);
    //digitalWrite(input2,HIGH);

  }
  if (but_id == 1) //used for relay
  {
    /*................service bat reply...................*/

    Serial.println("Relay operation");
    digitalWrite(input2, LOW);

    delay(350);
    digitalWrite(input2, HIGH);


  }

  /*  for (count=0; count<8; count++)
    {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(50);


    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(50);


    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, HIGH);
    delay(50);



    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    delay(50);


    }
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(50);


    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(50);*/
  //digitalWrite(in1, HIGH);
  //digitalWrite(in2, HIGH);
  //digitalWrite(in3, LOW);
  //digitalWrite(in4, LOW);
  //delay(20);
  //
  //
  //digitalWrite(in1, LOW);
  //digitalWrite(in2, HIGH);
  //digitalWrite(in3, HIGH);
  //digitalWrite(in4, LOW);
  //delay(20);

}
