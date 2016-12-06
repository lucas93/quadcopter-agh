// Motors.h

#ifndef _MOTORS_h
#define _MOTORS_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Servo\Servo.h"

class MotorsClass
{
public:
	static const int maxInput = 2000;
	static const int minInput = 1200;
	class Motor : public Servo
	{
	public:
		void set(int value)	//value between 0 : 1000
		{
			value = map(value, 0, 1000, minInput, maxInput);
			value = constrain(value, minInput, maxInput);
			writeMicroseconds(value);
		}
	};
	Motor motors[4];
	Motor *LF = &motors[1];		//left-front motor
	Motor *RF = &motors[2];		//right-front motor
	Motor *RB = &motors[3];		//right-back motor
	Motor *LB = &motors[0];		//left-back motor

	bool setup(int pin1, int pin2, int pin3, int pin4);

	void turn_off();	//  stops spinning motors				
	void writeMicroseconds_to_all(int value);	// write microseconds to all motors

	bool* get_arm_indicator_pointer();
};

extern MotorsClass Motors;

#endif

