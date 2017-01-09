// OrientationSensor.h

#ifndef _ORIENTATIONSENSOR_h
#define _ORIENTATIONSENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "RollPitchYaw.h"
#include "Led.h"
#include "mpu.h"

class OrientationSensorClass
{
 private:
	 RollPitchYaw *currentRPY = RollPitchYaw::get_current_RPY_pointer();
	 int ret;
	 float const roll_OFFSET = 0.0f;
	 float const pitch_OFFSET = 0.0f;

 public:
	 bool setup();
	 bool update_current_RPY();
};

extern OrientationSensorClass OrientationSensor;

#endif

