// 
// 
// 

#include "Controller.h"




ControllerClass Controller;

void ControllerClass::reset_PID_integrals()
{
}

bool ControllerClass::setup(int T_ms)
{
	K_Roll = EEPROM_variables.load(EEPROM_variables.K_Roll);
	Ti_Roll = EEPROM_variables.load(EEPROM_variables.Ti_Roll);
	Td_Roll = EEPROM_variables.load(EEPROM_variables.Td_Roll);
	windupIntegral_Roll = EEPROM_variables.load(EEPROM_variables.windupIntegral_Roll);

	K_Pitch = EEPROM_variables.load(EEPROM_variables.K_Pitch);
	Ti_Pitch = EEPROM_variables.load(EEPROM_variables.Ti_Pitch);
	Td_Pitch = EEPROM_variables.load(EEPROM_variables.Td_Pitch);
	windupIntegral_Pitch = EEPROM_variables.load(EEPROM_variables.windupIntegral_Pitch);

	K_YawRate = EEPROM_variables.load(EEPROM_variables.K_YawRate);
	Ti_YawRate = EEPROM_variables.load(EEPROM_variables.Ti_YawRate);
	Td_YawRate = EEPROM_variables.load(EEPROM_variables.Td_YawRate);
	windupIntegral_YawRate = EEPROM_variables.load(EEPROM_variables.windupIntegral_YawRate);

	is_armed = Motors.get_arm_indicator_pointer();
	RC_is_connected = RCReceiver.get_RC_connection_indicator_pointer();
	currentRPY = RollPitchYaw::get_current_RPY_pointer();
	targetRPY = RollPitchYaw::get_target_RPY_pointer();

	rollPID = PID(currentRPY->roll, rollOutput, 
		targetRPY->roll, K_Roll, Ti_Roll, Td_Roll, 
		windupIntegral_Roll, static_cast<float>(T_ms));
	pitchPID = PID(currentRPY->pitch, pitchOutput, 
		targetRPY->pitch, K_Pitch, Ti_Pitch, Td_Pitch, 
		windupIntegral_Pitch, static_cast<float>(T_ms));
	yawRatePID = PID(currentRPY->yawRate, yawRateOutput, 
		targetRPY->yawRate, K_YawRate, Ti_YawRate, Td_YawRate, 
		windupIntegral_YawRate, static_cast<float>(T_ms));

	*is_armed = false;
	return true;
}

void ControllerClass::control_loop()
{
	static bool justArmed = true;		//to clear PID timers just before calculating PID

	int TX_Throttle = RCReceiver.CH3;
	int ThrottleMin = RCReceiver.get_ThrottleMin();

	LF_Throttle = 0.0f;
	RF_Throttle = 0.0f;
	RB_Throttle = 0.0f;
	LB_Throttle = 0.0f;

	if (TX_Throttle < ThrottleMin)
	{
		*is_armed = false;
		Motors.turn_off();
		justArmed = true;

		return;
	}
	else
	{
		*is_armed = true;
		if (justArmed)
		{
			justArmed = false;
			reset_PID_integrals();
		}
	}

#if OPEN_LOOP_CONTROL
	rollOutput = K_Roll * targetRPY->roll;
	pitchOutput = K_Pitch * targetRPY->pitch;
	yawRateOutput = K_YawRate * targetRPY->yawRate;
#else
	rollPID.compute();
	pitchPID.compute();
	yawRatePID.compute();
#endif

	int throttleInput = map(TX_Throttle, RCReceiver.get_ThrottleMin(), RCReceiver.ch3Max, 0, maximumThrottleInput);
	throttleInput = constrain(throttleInput, 10, maximumThrottleInput);		//to make sure throttle is in boundries <0,1%  :  50%>

//#define print_param(param) Serial.print(String(#param " = ") + String(param) + "\t")
//#define println_param(param) Serial.println(String(#param " = ") + String(param) + "\t")
//
//	
//	print_param(targetRPY->roll);
//	print_param(maximumThrottleInput);
//	print_param(throttleInput);
//	println_param(rollOutput);
//
//#undef print_param()
//#undef println_param()

	LF_Throttle += throttleInput;
	RF_Throttle += throttleInput;
	RB_Throttle += throttleInput;
	LB_Throttle += throttleInput;

	// PITCH
	RB_Throttle -= pitchOutput;
	LB_Throttle -= pitchOutput;
	RF_Throttle += pitchOutput;
	LF_Throttle += pitchOutput;

	// ROLL
	RF_Throttle -= rollOutput;
	RB_Throttle -= rollOutput;
	LF_Throttle += rollOutput;
	LB_Throttle += rollOutput;

	// YAW_RATE
	LF_Throttle += yawRateOutput;
	RB_Throttle += yawRateOutput;
	RF_Throttle -= yawRateOutput;
	LB_Throttle -= yawRateOutput;

	//Motors.LF->set(0);
	//Motors.RF->set(0);
	//Motors.RB->set(0);
	//Motors.LB->set(0);
	Motors.LF->set(int(LF_Throttle + 0.5f));
	Motors.RF->set(int(RF_Throttle + 0.5f));
	Motors.RB->set(int(RB_Throttle + 0.5f));
	Motors.LB->set(int(LB_Throttle + 0.5f));

}
