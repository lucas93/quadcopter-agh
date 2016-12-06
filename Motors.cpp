// 
// 
// 

#include "Motors.h"




bool MotorsClass::setup(int pin1, int pin2, int pin3, int pin4)
{
	motors[0].attach(pin1);
	motors[1].attach(pin2);
	motors[2].attach(pin3);
	motors[3].attach(pin4);

	motors[0].writeMicroseconds(0);
	motors[1].writeMicroseconds(0);
	motors[2].writeMicroseconds(0);
	motors[3].writeMicroseconds(0);

	return true;
}


//  stops spinning motors
void MotorsClass::turn_off()
{
	motors[0].writeMicroseconds(1000);
	motors[1].writeMicroseconds(1000);
	motors[2].writeMicroseconds(1000);
	motors[3].writeMicroseconds(1000);
}


// write microseconds to all motors
void MotorsClass::writeMicroseconds_to_all(int value)
{
	motors[0].writeMicroseconds(value);
	motors[1].writeMicroseconds(value);
	motors[2].writeMicroseconds(value);
	motors[3].writeMicroseconds(value);
}

bool * MotorsClass::get_arm_indicator_pointer()
{
	static bool is_armed;
	return &is_armed;
}


MotorsClass Motors;