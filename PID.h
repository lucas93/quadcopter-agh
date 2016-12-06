// PID.h
#ifndef _PID_h
#define _PID_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class PID
{
 private:
	 float *PID_input;
	 float *PID_output;
	 float *PID_setpoint;

	 float *K, *Ti, *Td;
	 float KP, KI, KD;
	 float *windupGuard;

	 float previous_input;
	 float integral;
	 float T_s;	// period [s]

 public:
	 PID() = default;
	 PID(float &Input, float &Output, float &Setpoint, float &K, float &Ti, float &Td, float &windupIntegral, float T_ms);
	 void compute();
	 void integral_reset();
};

#endif

