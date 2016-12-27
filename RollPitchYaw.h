// RollPitchYaw.h

#ifndef _ROLLPITCHYAW_h
#define _ROLLPITCHYAW_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class RollPitchYaw
{
public:
	float roll, pitch, yaw, rollRate, pitchRate, yawRate;
	

	RollPitchYaw();
	RollPitchYaw(const RollPitchYaw& other);
	RollPitchYaw(float roll, float pitch, float yaw, float rollRate, float pitchRate, float yawRate);

	RollPitchYaw& operator =(RollPitchYaw& other);
	RollPitchYaw operator -(RollPitchYaw& other);

	static RollPitchYaw* get_current_RPY_pointer();
	static RollPitchYaw* get_target_RPY_pointer();

	void setRPY(const float roll, const float pitch, const float yaw, const float rollRate, const float pitchRate, const float yawRate);
};


#endif

