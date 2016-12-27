#include "EEPROM_variables.h"

String const EEPROM_variablesClass::var_name[] = {

	"ch1Min", "ch1Mean", "ch1Max",
	"ch2Min", "ch2Mean", "ch2Max",
	"ch3Min", "ch3Max",
	"ch4Min", "ch4Mean", "ch4Max",
	"ch5Min", "ch5Max",
	"ch6Min", "ch6Max",

	"K_Roll", "Ti_Roll", "Td_Roll", "windupIntegral_Roll",
	"K_Pitch", "Ti_Pitch", "Td_Pitch", "windupIntegral_Pitch",
	"K_YawRate", "Ti_YawRate", "Td_YawRate", "windupIntegral_YawRate"

	"K_RollRate", "Ti_RollRate", "Td_RollRate", "windupIntegral_RollRate",
	"K_PitchRate", "Ti_PitchRate", "Td_PitchRate", "windupIntegral_PitchRate"
};


bool EEPROM_variablesClass::setup()
{
	return true;
}


// save parameter to EEPROM
void EEPROM_variablesClass::save(int var_idx, float var)
{
	EEPROM.put(var_idx * sizeof(float), var);
}

void EEPROM_variablesClass::save(var var_idx, float var)
{
	save(static_cast<int>(var_idx), var);
}



// load parameter from EEPROM to &var with parameter index
float EEPROM_variablesClass::load(int var_idx)
{
	float var;
	EEPROM.get(var_idx * sizeof(float), var);
	return var;
}

EEPROM_variablesClass EEPROM_variables;