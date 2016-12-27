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

	//FIRST WORD IN INPUT
	if (input[0] == "set")		// set parameter value
	{
		if (size == 3 && input[2].toFloat() >= 0.0)
		{
			if (input[1] == "K_Roll")
			{
				var_pointer = &(Controller.K_Roll);
				var_enum = EEPROM_variables.K_Roll;
			}
			if (input[1] == "K_RollRate")
			{
				var_pointer = &(Controller.K_RollRate);
				var_enum = EEPROM_variables.K_RollRate;
			}
			else if (input[1] == "K_Pitch")
			{
				var_pointer = &(Controller.K_Pitch);
				var_enum = EEPROM_variables.K_Pitch;
			}
			else if (input[1] == "K_PitchRate")
			{
				var_pointer = &(Controller.K_PitchRate);
				var_enum = EEPROM_variables.K_PitchRate;
			}
			else if (input[1] == "K_YawRate")
			{
				var_pointer = &(Controller.K_YawRate);
				var_enum = EEPROM_variables.K_YawRate;
			}
			else if (input[1] == "Ti_Roll")
			{
				var_pointer = &(Controller.Ti_Roll);
				var_enum = EEPROM_variables.Ti_Roll;
			}
			else if (input[1] == "Ti_RollRate")
			{
				var_pointer = &(Controller.Ti_RollRate);
				var_enum = EEPROM_variables.Ti_RollRate;
			}
			else if (input[1] == "Ti_Pitch")
			{
				var_pointer = &(Controller.Ti_Pitch);
				var_enum = EEPROM_variables.Ti_Pitch;
			}
			else if (input[1] == "Ti_PitchRate")
			{
				var_pointer = &(Controller.Ti_PitchRate);
				var_enum = EEPROM_variables.Ti_PitchRate;
			}
			else if (input[1] == "Ti_YawRate")
			{
				var_pointer = &(Controller.Ti_YawRate);
				var_enum = EEPROM_variables.Ti_YawRate;
			}
			else if (input[1] == "Td_Roll")
			{
				var_pointer = &(Controller.Td_Roll);
				var_enum = EEPROM_variables.Td_Roll;
			}
			else if (input[1] == "Td_RollRate")
			{
				var_pointer = &(Controller.Td_RollRate);
				var_enum = EEPROM_variables.Td_RollRate;
			}
			else if (input[1] == "Td_Pitch")
			{
				var_pointer = &(Controller.Td_Pitch);
				var_enum = EEPROM_variables.Td_Pitch;
			}
			else if (input[1] == "Td_PitchRate")
			{
				var_pointer = &(Controller.Td_PitchRate);
				var_enum = EEPROM_variables.Td_PitchRate;
			}
			else if (input[1] == "Td_YawRate")
			{
				var_pointer = &(Controller.Td_YawRate);
				var_enum = EEPROM_variables.Td_YawRate;
			}
			else if (input[1] == "windupIntegral_Roll")
			{
				var_pointer = &(Controller.windupIntegral_Roll);
				var_enum = EEPROM_variables.windupIntegral_Roll;
			}
			else if (input[1] == "windupIntegral_RollRate")
			{
				var_pointer = &(Controller.windupIntegral_RollRate);
				var_enum = EEPROM_variables.windupIntegral_RollRate;
			}
			else if (input[1] == "windupIntegral_Pitch")
			{
				var_pointer = &(Controller.windupIntegral_Pitch);
				var_enum = EEPROM_variables.windupIntegral_Pitch;
			}
			else if (input[1] == "windupIntegral_PitchRate")
			{
				var_pointer = &(Controller.windupIntegral_PitchRate);
				var_enum = EEPROM_variables.windupIntegral_PitchRate;
			}
			else if (input[1] == "windupIntegral_YawRate")
			{
				var_pointer = &(Controller.windupIntegral_YawRate);
				var_enum = EEPROM_variables.windupIntegral_YawRate;
			}
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
			if (input[1] == "K_Roll")
			{
				var_pointer = &(Controller.K_Roll);
				var_enum = EEPROM_variables.K_Roll;
			}
			if (input[1] == "K_RollRate")
			{
				var_pointer = &(Controller.K_RollRate);
				var_enum = EEPROM_variables.K_RollRate;
			}
			else if (input[1] == "K_Pitch")
			{
				var_pointer = &(Controller.K_Pitch);
				var_enum = EEPROM_variables.K_Pitch;
			}
			else if (input[1] == "K_PitchRate")
			{
				var_pointer = &(Controller.K_PitchRate);
				var_enum = EEPROM_variables.K_PitchRate;
			}
			else if (input[1] == "K_YawRate")
			{
				var_pointer = &(Controller.K_YawRate);
				var_enum = EEPROM_variables.K_YawRate;
			}
			else if (input[1] == "Ti_Roll")
			{
				var_pointer = &(Controller.Ti_Roll);
				var_enum = EEPROM_variables.Ti_Roll;
			}
			else if (input[1] == "Ti_RollRate")
			{
				var_pointer = &(Controller.Ti_RollRate);
				var_enum = EEPROM_variables.Ti_RollRate;
			}
			else if (input[1] == "Ti_Pitch")
			{
				var_pointer = &(Controller.Ti_Pitch);
				var_enum = EEPROM_variables.Ti_Pitch;
			}
			else if (input[1] == "Ti_PitchRate")
			{
				var_pointer = &(Controller.Ti_PitchRate);
				var_enum = EEPROM_variables.Ti_PitchRate;
			}
			else if (input[1] == "Ti_YawRate")
			{
				var_pointer = &(Controller.Ti_YawRate);
				var_enum = EEPROM_variables.Ti_YawRate;
			}
			else if (input[1] == "Td_Roll")
			{
				var_pointer = &(Controller.Td_Roll);
				var_enum = EEPROM_variables.Td_Roll;
			}
			else if (input[1] == "Td_RollRate")
			{
				var_pointer = &(Controller.Td_RollRate);
				var_enum = EEPROM_variables.Td_RollRate;
			}
			else if (input[1] == "Td_Pitch")
			{
				var_pointer = &(Controller.Td_Pitch);
				var_enum = EEPROM_variables.Td_Pitch;
			}
			else if (input[1] == "Td_PitchRate")
			{
				var_pointer = &(Controller.Td_PitchRate);
				var_enum = EEPROM_variables.Td_PitchRate;
			}
			else if (input[1] == "Td_YawRate")
			{
				var_pointer = &(Controller.Td_YawRate);
				var_enum = EEPROM_variables.Td_YawRate;
			}
			else if (input[1] == "windupIntegral_Roll")
			{
				var_pointer = &(Controller.windupIntegral_Roll);
				var_enum = EEPROM_variables.windupIntegral_Roll;
			}
			else if (input[1] == "windupIntegral_RollRate")
			{
				var_pointer = &(Controller.windupIntegral_RollRate);
				var_enum = EEPROM_variables.windupIntegral_RollRate;
			}
			else if (input[1] == "windupIntegral_Pitch")
			{
				var_pointer = &(Controller.windupIntegral_Pitch);
				var_enum = EEPROM_variables.windupIntegral_Pitch;
			}
			else if (input[1] == "windupIntegral_PitchRate")
			{
				var_pointer = &(Controller.windupIntegral_PitchRate);
				var_enum = EEPROM_variables.windupIntegral_PitchRate;
			}
			else if (input[1] == "windupIntegral_YawRate")
			{
				var_pointer = &(Controller.windupIntegral_YawRate);
				var_enum = EEPROM_variables.windupIntegral_YawRate;
			}
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
