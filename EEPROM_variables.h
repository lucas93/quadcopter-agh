// EEPROM_variables.h

#ifndef _EEPROM_VARIABLES_h
#define _EEPROM_VARIABLES_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "EEPROM\EEPROM.h"

class EEPROM_variablesClass
{
public:
	bool setup();
	static String const var_name[];
	enum var		//names of variables stored on EEPROM
	{
		ch1Min = 0, ch1Middle, ch1Max,
		ch2Min, ch2Middle, ch2Max,
		ch3Min, ch3Max,
		ch4Min, ch4Middle, ch4Max,
		ch5Min, ch5Max,
		ch6Min, ch6Max,

		K_Roll, Ti_Roll, Td_Roll, windupIntegral_Roll,
		K_Pitch, Ti_Pitch, Td_Pitch, windupIntegral_Pitch,
		K_YawRate, Ti_YawRate, Td_YawRate, windupIntegral_YawRate,

		K_RollRate, Ti_RollRate, Td_RollRate, windupIntegral_RollRate,
		K_PitchRate, Ti_PitchRate, Td_PitchRate, windupIntegral_PitchRate,

	};

	void save(int var_idx, float var);	// save parameter var to EEPROM	with parameter index
	void save(var var_idx, float var);	// save parameter var to EEPROM	with parameter index
	float load(int var_idx);				// load parameter from EEPROM to &var with parameter index

};

extern EEPROM_variablesClass EEPROM_variables;

#endif

