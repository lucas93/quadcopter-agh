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
	float KP_Roll;
	float KI_Roll;
	float KD_Roll;
	float windupIntegral_Roll;

	float KP_RollRate;
	float KI_RollRate;
	float KD_RollRate;
	float windupIntegral_RollRate;

	float KP_Pitch;
	float KI_Pitch;
	float KD_Pitch;
	float windupIntegral_Pitch;

	float KP_PitchRate;
	float KI_PitchRate;
	float KD_PitchRate;
	float windupIntegral_PitchRate;

	float KP_YawRate;
	float KI_YawRate;
	float KD_YawRate;
	float windupIntegral_YawRate;

	void reset_PID_integrals();
	bool setup(int T_ms);
	void control_loop();
};

extern ControllerClass Controller;

#endif

