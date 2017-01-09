// Controller.h

#ifndef _CONTROLLER_h
#define _CONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID.h"
#include "RollPitchYaw.h"
#include "Motors.h"
#include "RCReceiver.h"

#define OPEN_LOOP_CONTROL false		// change between closed-loop and open-loop control

class ControllerClass
{
private:
	PID rollPID, pitchPID, rollRatePID, pitchRatePID, yawRatePID;
	RollPitchYaw *currentRPY, *targetRPY;
	bool *is_armed, *RC_is_connected;

	float rollAngleOutput, pitchAngleOutput, rollRateOutput, pitchRateOutput, yawRateOutput;		//PID outputs
	int maximumThrottleOutput = 500;	// Throttle can account for maximum of 50% of maximum motor power
	float LF_Throttle;		// negative pitch rotation, negative yawRate rotation
	float RF_Throttle;		// negative pitch rotation, positive yawRate rotation
	float RB_Throttle;		// positive pitch rotation, negative yawRate rotation
	float LB_Throttle;		// positive pitch rotation, positive yawRate rotation

public:
	float K_Roll;
	float Ti_Roll;
	float Td_Roll;
	float windupIntegral_Roll;

	float K_RollRate;
	float Ti_RollRate;
	float Td_RollRate;
	float windupIntegral_RollRate;

	float K_Pitch;
	float Ti_Pitch;
	float Td_Pitch;
	float windupIntegral_Pitch;

	float K_PitchRate;
	float Ti_PitchRate;
	float Td_PitchRate;
	float windupIntegral_PitchRate;

	float K_YawRate;
	float Ti_YawRate;
	float Td_YawRate;
	float windupIntegral_YawRate;

	void reset_PID_integrals();
	bool setup(int T_ms);
	void control_loop();
};

extern ControllerClass Controller;

#endif

