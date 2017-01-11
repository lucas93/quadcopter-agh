// 
// 
// 

#include "SerialInputProcesser.h"


bool SerialInputProcesserClass::setup()
{
	is_armed = Motors.get_arm_indicator_pointer();
	return true;
}

void SerialInputProcesserClass::proccess_serial_if_any_and_unarmed()
{
	if (!Serial.available() || *is_armed) return;	// return if quad is armed or no serial input available

	String input[10];		// 10 for max number of words in serial read
	uint8_t size = 0;
	float * var_pointer = nullptr;		// pointer to variable to set
	float var_value = 0.0;				// value of variable to set
	EEPROM_variablesClass::var var_enum;			// enum of variable to change

	while (Serial.available())		// read from serial
	{
		input[size] = Serial.readStringUntil(' ');
		size++;
	}

	if (size < 2)	return;		// too few arguments

#define POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(PARAMETER) if (input[1] == #PARAMETER)\
	{\
		var_pointer = &(Controller.PARAMETER);\
		var_enum = EEPROM_variables.PARAMETER;\
	}\

	//FIRST WORD IN INPUT
	if (input[0] == "set")		// set parameter value
	{
		if (size == 3 && input[2].toFloat() >= 0.0)
		{
			POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KP_Roll)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KP_RollRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KP_Pitch)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KP_PitchRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KP_YawRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KI_Roll)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KI_RollRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KI_Pitch)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KI_PitchRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KI_YawRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KD_Roll)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KD_RollRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KD_Pitch)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KD_PitchRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KD_YawRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(windupIntegral_Roll)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(windupIntegral_RollRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(windupIntegral_Pitch)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(windupIntegral_PitchRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(windupIntegral_YawRate)
			else
			{
				Serial.println("ERROR setting parameter!!");
				return;
			}
			*var_pointer = input[2].toFloat();
			EEPROM_variables.save(var_enum, *var_pointer);

			Serial.print("Set ");
			Serial.print(EEPROM_variables.var_name[static_cast<int>(var_enum)]);
			Serial.print("\t");
			Serial.println(*var_pointer);
		}
		else
		{
			Serial.println("ERROR setting parameter!!");
			return;
		}
	}
	else if (input[0] == "get")
	{
		if (size == 2)
		{
			

			POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KP_Roll)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KP_RollRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KP_Pitch)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KP_PitchRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KP_YawRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KI_Roll)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KI_RollRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KI_Pitch)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KI_PitchRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KI_YawRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KD_Roll)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KD_RollRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KD_Pitch)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KD_PitchRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(KD_YawRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(windupIntegral_Roll)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(windupIntegral_RollRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(windupIntegral_Pitch)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(windupIntegral_PitchRate)
			else POINT_ON_PARAMETER_IF_IT_IS_GIVEN_WITH_INPUT(windupIntegral_YawRate)
			else
			{
				Serial.println("ERROR getting parameter!!!");
				return;
			}

			Serial.print("Current ");
			Serial.print(EEPROM_variables.var_name[static_cast<int>(var_enum)]);
			Serial.print("\t");
			Serial.println(*var_pointer);
		}
		else
		{
			Serial.println("ERROR getting parameter!!!");
			return;
		}
	}
	else
	{
		Serial.println("ERROR in serial communication!!!");
		return;
	}
}

SerialInputProcesserClass SerialInputProcesser;
