// RCtoRPY.h

#ifndef _RCTORPY_h
#define _RCTORPY_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "RollPitchYaw.h"
#include "Motors.h"
#include "RCReceiver.h"

class RCtoRPYClass
{
 private:
	 bool *is_armed;			// indicates if drone is armed
	 bool *RC_is_connected;		// indicates if RC is connected
	 const float angle_max = 20.00;		// maximum input value for set angle (roll or pitch)
	 const float angle_rate_max = 50.0;			// maximum input value for set angle rate (yawRate)
	 RollPitchYaw *target_RPY;

 public:
	 bool setup();
	 bool update_target_RPY();
};

extern RCtoRPYClass RCtoRPY;

#endif

