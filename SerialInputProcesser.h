// SerialInputProcesser.h

#ifndef _SERIALINPUTPROCESSER_h
#define _SERIALINPUTPROCESSER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


#include "Motors.h"
#include "Controller.h"
class SerialInputProcesserClass
{
private:
	bool* is_armed;		// indicates if drone is armed


public:
	bool setup();
	void proccess_serial_if_any_and_unarmed();
};

extern SerialInputProcesserClass SerialInputProcesser;

#endif

