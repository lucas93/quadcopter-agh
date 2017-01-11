#include "EEPROM_variables.h"

String const EEPROM_variablesClass::var_name[] = {

	"ch1Min", "ch1Mean", "ch1Max",
	"ch2Min", "ch2Mean", "ch2Max",
	"ch3Min", "ch3Max",
	"ch4Min", "ch4Mean", "ch4Max",
	"ch5Min", "ch5Max",
	"ch6Min", "ch6Max",

	"KP_Roll", "KI_Roll", "KD_Roll", "windupIntegral_Roll",
	"KP_Pitch", "KI_Pitch", "KD_Pitch", "windupIntegral_Pitch",
	"KP_YawRate", "KI_YawRate", "KD_YawRate", "windupIntegral_YawRate",

	"KP_RollRate", "KI_RollRate", "KD_RollRate", "windupIntegral_RollRate",
	"KP_PitchRate", "KI_PitchRate", "KD_PitchRate", "windupIntegral_PitchRate"
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