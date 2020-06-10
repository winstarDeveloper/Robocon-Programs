#ifndef dcServo_h
#define dcServo_h

#include <Wire.h>

	// set the desired speed of robot
#define dampConst 2
			// 1 : forward, 0 : reverse
#define setPidVal 5

class dcServo{
	private:
		int address;
	public:

	dcServo(int);
	void LineForward( int );
	void LineReverse(int );
	void Forward( int );
	void Reverse(int );
	void Stop(int  );
	void Position(int , int, int);
	void setPID(double , double, double);
	void setdefaultPID(void);
	void setOnSpot(int lineEndCount, int endCount, int speeed);
//	void getPID();
//	void setMaxSpeed(int);
};

#endif