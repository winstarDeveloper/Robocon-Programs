#include <dcServo.h>
#include<Wire.h>
dcServo motor(8);//Create an object motor with address 8

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  motor.setMaxSpeed(80);//Set maximum speed of motor
}

void loop() {
  // put your main code here, to run repeatedly:
  //obj.Position(int count, int runSpeed , int dir );
  //  count->pulse from encoder
  //  runspeed-> motor speed
  //  dir-> 1=forward 2=reverse

  motor.Position(5, 60, 1);
  delay(5000);
  motor.Position(3, 40, 2);
  delay(5000);

}
