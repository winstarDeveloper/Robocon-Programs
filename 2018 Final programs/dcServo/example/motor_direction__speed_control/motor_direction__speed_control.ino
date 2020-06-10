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

  motor.Forward(70);//motor forward at 70 rpm
  delay(5000);
  motor.Stop(50);//motor stop at 50 damping (damping 0 to 255)
  delay(5000);
  motor.Reverse(50);//motor reverse at 70 rpm
  delay(5000);
  motor.Stop(30);//motor stop at 30 damping (damping 0 to 255)
  delay(5000);

}
