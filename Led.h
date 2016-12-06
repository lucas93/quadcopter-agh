// Led.h

#ifndef _LED_h
#define _LED_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class LedClass
{
private:
	int led_pin;
	bool state;

public:
	bool setup(int _led_pin = 13);
	void on();		// force led on
	void off();		// force led off
	void toggle();	// force toggle
	void blink(int howManyTimes = 2, int howLong = 200);	// blinks led howManyTimes for howLong milliseconds
	void blink_frequently(int interval);					// start blinking with given interval, has to be run frequently
};

extern LedClass Led;

#endif

