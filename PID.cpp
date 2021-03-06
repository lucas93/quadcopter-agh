// 
// 
// 

#include "PID.h"


PID::PID(float &Input, float &Output, float &Setpoint, float &K, float &Ki, float &Kd, float &windupIntegral, float T_ms) : T_s(T_ms/1000)
{
	previous_input = 0.0;
	integral = 0.0;

	PID_input = &Input;
	PID_output = &Output;
	PID_setpoint = &Setpoint;

	this->K = &K;
	this->Ki = &Ki;
	this->Kd = &Kd;

	KP = K;
	KI = Ki;
	KD = K * Kd;

	windupGuard = &windupIntegral;
}
void PID::compute()
{
	float error = *PID_setpoint - *PID_input;
	integral = constrain(integral + error , -*windupGuard, *windupGuard);
	float derivative = (*PID_input - previous_input) ;	//derivative is negative to substract it in pid loop

	*PID_output = KP * error + KI * integral + KD * derivative;

	previous_input = *PID_input;
}

void PID::integral_reset()
{
	integral = 0.0;
}
