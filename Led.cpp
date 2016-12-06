#include "Led.h"


bool LedClass::setup(int _led_pin) 
{
	this->led_pin = _led_pin;
	state = false;
	pinMode(led_pin, OUTPUT);
	digitalWrite(led_pin, state);
	return true;
}


// force led on once
void LedClass::on()
{
	digitalWrite(led_pin, state = true);
}


// force led off once
void LedClass::off()
{
	digitalWrite(led_pin, state = false);
}


// force toggle once
void LedClass::toggle()
{
	digitalWrite(led_pin, state = !state);
}


// blinks led howManyTimes for howLong milliseconds
void LedClass::blink(int howManyTimes, int howLong)
{
	for (int i = 0; i < howManyTimes; i++)
	{
		on();
		delay(howLong);
		off();
		delay(howLong);
		digitalWrite(led_pin, state);
	}
}


// frequent blinking with given interval, has to be run frequently
void LedClass::blink_frequently(int interval)
{
	static long long t = millis();

	if (t + interval < millis())
	{
		toggle();
		t = millis();
	}

}

LedClass Led;