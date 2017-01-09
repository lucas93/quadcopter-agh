// 
// 
// 

#include "RCtoRPY.h"

RCtoRPYClass RCtoRPY;

static float map(float value, float fromLow, float fromHigh, float toLow, float toHigh)
{
	if (fromLow == fromHigh)
		return 0.0f;

	float a = (toLow - toHigh) / (fromLow - fromHigh);
	float b = toLow - a * fromLow;

	return a * value + b;
}

bool RCtoRPYClass::setup()
{
	is_armed = Motors.get_arm_indicator_pointer();
	RC_is_connected = RCReceiver.get_RC_connection_indicator_pointer();
	target_RPY = RollPitchYaw::get_target_RPY_pointer();
	return true;
}

bool RCtoRPYClass::update_target_RPY()
{
	if (*RC_is_connected == false || *is_armed == false)		//throttle is below min or lost communication
	{
		target_RPY->setRPY(0, 0, 0, 0, 0, 0);	// so set targetRPY to zero to ensure quad is stedy
		return false;
	}

	//for convinience
	const int ch1Max = RCReceiver.ch1Max;	
	const int ch1Middle = RCReceiver.ch1Middle;
	const int ch1Min = RCReceiver.ch1Min;
	const int ch2Max = RCReceiver.ch2Max;
	const int ch2Middle = RCReceiver.ch2Middle;
	const int ch2Min = RCReceiver.ch2Min;
	const int ch3Max = RCReceiver.ch3Max;
	const int ch3Min = RCReceiver.ch3Min;
	const int ch4Max = RCReceiver.ch4Max;
	const int ch4Middle = RCReceiver.ch4Middle;
	const int ch4Min = RCReceiver.ch4Min;
	
	const int CH1 = RCReceiver.CH1;
	const int CH2 = RCReceiver.CH2;
	const int CH3 = RCReceiver.CH3;
	const int CH4 = RCReceiver.CH4;


	float r, p, y,rr, pr, yr;		//roll, pitch, yaw, yawRate
	y = 0;	//not calculating yaw
	rr = 0;	//not calculating rollRate
	pr = 0;	//not calculating pitchRate

	if (CH1 >= ch1Middle)			//roll is connected to ch1
		r = map(CH1, ch1Middle, ch1Max, 0, angle_max);
	else
		r = map(CH1, ch1Min, ch1Middle, -angle_max, 0);

	if (CH2 >= ch2Middle)			//pitch is connected to ch2
		p = map(CH2, ch2Middle, ch2Max, 0, angle_max);
	else
		p = map(CH2, ch2Min, ch2Middle, -angle_max, 0);

	if (CH4 >= ch4Middle)			//yawRate is connected to ch4
		yr = map(CH4, ch4Middle, ch4Max, 0, angle_rate_max);
	else
		yr = map(CH4, ch4Min, ch4Middle, -angle_rate_max, 0);

	r *= -1.0;	// dependent on base frame
	p *= -1.0;	// dependent on base frame
	yr *= -1.0;	// dependent on base frame

	//make sure angles are in boundries <-ANGLE_MAX; ANGLE_MAX>
	r = constrain(r, -angle_max, angle_max);
	p = constrain(p, -angle_max, angle_max);
	yr = constrain(yr, -angle_rate_max, angle_rate_max);

	target_RPY->setRPY(r, p, y, rr, pr, yr);
	return true;
}


