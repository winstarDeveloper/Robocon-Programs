#include "dcServo.h"
#include "Arduino.h"
#include <Wire.h>

dcServo::dcServo(int a){
	Wire.begin();
	address=a;
}

void dcServo::LineForward(int runSpeed = 0){
	Wire.beginTransmission(address);
	Wire.write(5);
	if(runSpeed > 255)
		Wire.write(255);
	else
		Wire.write(runSpeed);
	Wire.flush();
	Wire.endTransmission();
	Wire.flush();
}

void dcServo::LineReverse(int runSpeed = 0){
	Wire.beginTransmission(address);
	Wire.write(6);
	if(runSpeed > 255)
		Wire.write(255);
	else
		Wire.write(runSpeed);
	Wire.flush();
	Wire.endTransmission();
	Wire.flush();
}


void dcServo::Forward(int runSpeed){
	Wire.beginTransmission(address);
	Wire.write(1);
	if(runSpeed < 255){
		Wire.write(runSpeed);
		Wire.write(0);
	}else if(runSpeed > 255 && runSpeed < 500){
		Wire.write(255);
		Wire.write(runSpeed);
	}
	/*
	while(runSpeed > 244){
		Wire.write(244);
		runSpeed = runSpeed - 244;
	}
	Wire.write(runSpeed);
	*/
	Wire.flush();
	Wire.endTransmission();
	Wire.flush();
}

void dcServo::Reverse(int runSpeed){
	Wire.beginTransmission(address);
	Wire.write(2);
	if(runSpeed < 255){
		Wire.write(runSpeed);
		Wire.write(0);
	}else if(runSpeed > 255 && runSpeed < 500){
		Wire.write(255);
		Wire.write(runSpeed);
	}
	/*
	while(runSpeed > 244){
		Wire.write(244);
		runSpeed = runSpeed - 244;
	}
	Wire.write(runSpeed);
	*/
	Wire.flush();
	Wire.endTransmission();
	Wire.flush();
}

void dcServo::Stop(int damp ){
	Wire.beginTransmission(address);
	Wire.write(0);
	Wire.write(damp);
//	Wire.write(motorDir);
	Wire.flush();
	Wire.endTransmission();
	Wire.flush();
}

void dcServo :: Position (int dir, int posCount, int runSpeed){
	Wire.beginTransmission(address);
	Wire.write(3);
	Wire.write(dir);
	Wire.write(runSpeed);
	
	// posCount max value is 566
	if(posCount < 255){
		Wire.write(posCount);
		Wire.write(0);
		Wire.write(0);
	}else if(posCount > 255 && posCount < 500){
		Wire.write(255);
		Wire.write(posCount - 255);
		Wire.write(0);
	}else{
		Wire.write(255);
		Wire.write(255);
		Wire.write(posCount - 500);
	}
	/*
	while(posCount > 244){
		Wire.write(244);
		posCount = posCount - 244;
	}
	Wire.write(posCount);
	*/
	Wire.flush();
	Wire.endTransmission();
	Wire.flush();
}

void dcServo :: setPID(double pa, double in, double de){
	/*
	int p = pa * 1000, i = in * 1000, d = de * 1000;
	Wire.beginTransmission(address);
	Wire.write(4);
	while(p > 244){
		Wire.write(244);
		p = p - 244;
	}
	Wire.write(p);
	Wire.write(0);
	while(i > 244){
		Wire.write(244);
		i = i - 244;
	}
	Wire.write(i);
	Wire.write(0);
	while(d > 244){
		Wire.write(244);
		d = d - 244;
	}
	Wire.write(d);
	Wire.endTransmission();
	*/
}

void dcServo :: setdefaultPID(){
/*
	Wire.beginTransmission(address);
	Wire.write(7);
	Wire.endTransmission();
*/
}

void dcServo :: setOnSpot(int lineEndCount, int endCount, int speeed){
	Wire.beginTransmission(address);
	Wire.write(8);
	Wire.write(lineEndCount);
	Wire.write(endCount);
	Wire.write(speeed);
	Wire.flush();
	Wire.endTransmission();
	Wire.flush();
}

/*
void dcServo:: setMaxSpeed(int maxSpeed){
	Wire.beginTransmission(address);
	Wire.write(motorMaxSpeed);
	Wire.write(maxSpeed);
	Wire.endTransmission();
}
*/