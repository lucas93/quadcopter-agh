#include "RCReceiver.h"
#include "Motors.h"
#include "Buzzer.h"
#include "EEPROM_variables.h"
#include "OrientationSensor.h"



void RCReceiverClass::ch6_count()
{
	if (digitalRead(RCReceiver.ch6_pin) == HIGH)	RCReceiver.ch6_start = micros();
	else
	{
		RCReceiver.ch6_global_count = (uint16_t)(micros() - RCReceiver.ch6_start);
		RCReceiver.flag = HIGH;
		RCReceiver.ch6_global_flag = HIGH;
	}
}

void RCReceiverClass::ch5_count()
{
	if (digitalRead(RCReceiver.ch5_pin) == HIGH)	RCReceiver.ch5_start = micros();
	else
	{
		RCReceiver.ch5_global_count = (uint16_t)(micros() - RCReceiver.ch5_start);
		RCReceiver.flag = HIGH;
		RCReceiver.ch5_global_flag = HIGH;
	}
}

void RCReceiverClass::ch4_count()
{
	if (digitalRead(RCReceiver.ch4_pin) == HIGH)	RCReceiver.ch4_start = micros();
	else
	{
		RCReceiver.ch4_global_count = (uint16_t)(micros() - RCReceiver.ch4_start);
		RCReceiver.flag = HIGH;
		RCReceiver.ch4_global_flag = HIGH;
	}
}

void RCReceiverClass::ch3_count()
{
	if (digitalRead(RCReceiver.ch3_pin) == HIGH)	RCReceiver.ch3_start = micros();
	else
	{
		RCReceiver.ch3_global_count = (uint16_t)(micros() - RCReceiver.ch3_start);
		RCReceiver.flag = HIGH;
		RCReceiver.ch3_global_flag = HIGH;
	}
}

void RCReceiverClass::ch2_count()
{
	if (digitalRead(RCReceiver.ch2_pin) == HIGH)	RCReceiver.ch2_start = micros();
	else
	{
		RCReceiver.ch2_global_count = (uint16_t)(micros() - RCReceiver.ch2_start);
		RCReceiver.flag = HIGH;
		RCReceiver.ch2_global_flag = HIGH;
	}
}

void RCReceiverClass::ch1_count()
{
	if (digitalRead(RCReceiver.ch1_pin) == HIGH)	RCReceiver.ch1_start = micros();
	else
	{
		RCReceiver.ch1_global_count = (uint16_t)(micros() - RCReceiver.ch1_start);
		RCReceiver.flag = HIGH;
		RCReceiver.ch1_global_flag = HIGH;
	}
}

void RCReceiverClass::setup()
{
	ch1Min = (int)EEPROM_variables.load(EEPROM_variables.ch1Min);
	ch1Middle = (int)EEPROM_variables.load(EEPROM_variables.ch1Middle);
	ch1Max = (int)EEPROM_variables.load(EEPROM_variables.ch1Max);

	ch2Min = (int)EEPROM_variables.load(EEPROM_variables.ch2Min);
	ch2Middle = (int)EEPROM_variables.load(EEPROM_variables.ch2Middle);
	ch2Max = (int)EEPROM_variables.load(EEPROM_variables.ch2Max);

	ch3Min = (int)EEPROM_variables.load(EEPROM_variables.ch3Min);
	ch3Max = (int)EEPROM_variables.load(EEPROM_variables.ch3Max);

	ch4Min = (int)EEPROM_variables.load(EEPROM_variables.ch4Min);
	ch4Middle = (int)EEPROM_variables.load(EEPROM_variables.ch4Middle);
	ch4Max = (int)EEPROM_variables.load(EEPROM_variables.ch4Max);

	ThrottleMin = ch3Min + ThrottleBuffer;
	ThrottleMax = ch3Max;

	pinMode(ch6_pin, INPUT);
	pinMode(ch5_pin, INPUT);
	pinMode(ch4_pin, INPUT);
	pinMode(ch3_pin, INPUT);
	pinMode(ch2_pin, INPUT);
	pinMode(ch1_pin, INPUT);

	attachInterrupt(ch6_pin, ch6_count, CHANGE);
	attachInterrupt(ch5_pin, ch5_count, CHANGE);
	attachInterrupt(ch4_pin, ch4_count, CHANGE);
	attachInterrupt(ch3_pin, ch3_count, CHANGE);
	attachInterrupt(ch2_pin, ch2_count, CHANGE);
	attachInterrupt(ch1_pin, ch1_count, CHANGE);

	RC_is_connected = get_RC_connection_indicator_pointer();
	motors_is_armed = Motors.get_arm_indicator_pointer();
	*RC_is_connected = false;
}
bool * RCReceiverClass::get_RC_connection_indicator_pointer()
{
	static bool RC_is_connected;
	return &RC_is_connected;
}

int RCReceiverClass::get_ThrottleMin()
{
	return ThrottleMin;
}


// returns false when connection is lost
bool RCReceiverClass::update_channels()
{
	volatile static uint8_t updateflags;//lcoal flag
	volatile static uint8_t ch6_update_flag;//lcoal flag
	volatile static uint8_t ch5_update_flag;//lcoal flag
	volatile static uint8_t ch4_update_flag;//lcoal flag
	volatile static uint8_t ch3_update_flag;//lcoal flag
	volatile static uint8_t ch2_update_flag;//lcoal flag
	volatile static uint8_t ch1_update_flag;//lcoal flag

	static int n = 0;			//counter to check communication
	static auto tn = millis();	//counter to check communication

	if (flag)
	{
		n++;				//increment connection check counter
		noInterrupts();
		updateflags = flag;
		ch6_update_flag = ch6_global_flag;
		ch5_update_flag = ch5_global_flag;
		ch4_update_flag = ch4_global_flag;
		ch3_update_flag = ch3_global_flag;
		ch2_update_flag = ch2_global_flag;
		ch1_update_flag = ch1_global_flag;

		if (ch6_update_flag)
		{
			CH6 = ch6_global_count;
		}
		if (ch5_update_flag)
		{
			CH5 = ch5_global_count;
		}
		if (ch4_update_flag)
		{
			CH4 = ch4_global_count;
		}
		if (ch3_update_flag)
		{
			CH3 = ch3_global_count;
		}
		if (ch2_update_flag)
		{
			CH2 = ch2_global_count;
		}
		if (ch1_update_flag)
		{
			CH1 = ch1_global_count;
		}

		ch6_global_count = 0;
		ch6_global_flag = 0;
		ch6_update_flag = 0;

		ch5_global_count = 0;
		ch5_global_flag = 0;
		ch5_update_flag = 0;

		ch4_global_count = 0;
		ch4_global_flag = 0;
		ch4_update_flag = 0;

		ch3_global_count = 0;
		ch3_global_flag = 0;
		ch3_update_flag = 0;

		ch2_global_count = 0;
		ch2_global_flag = 0;
		ch2_update_flag = 0;

		ch1_global_count = 0;
		ch1_global_flag = 0;
		ch1_update_flag = 0;
		flag = 0;
		interrupts();

		return true;
	}
	else
	{
		if (tn + 100 < millis())		//check communication
		{
			tn = millis();
			if (n == 0)					//no communication for last 100ms, so lost communication
			{
				CH1 = 10;
				CH2 = 10;
				CH3 = 10;
				CH4 = 10;
				CH5 = 10;
				CH6 = 10;
				*RC_is_connected = false;
				return false;
			}
			else
			{
				n = 0;
				*RC_is_connected = true;
				return true;
			}
		}
		return true;
	}
}

void RCReceiverClass::calibrate_RC_if_ready_and_not_armed()
{
	static auto calibrated = false;
	// calibrate RC if motors are not armed and CH5 and CH6 are turned to min
	if (/**motors_is_armed*/calibrated == false &&  ch5Max - 10 < CH5  && ch6Max - 10 < CH6)
	{
		//finding min, max and mean values
		int ch1Min = 1506, ch1Middle = 1506, ch1Max = 1506;	//max is knob left
		int ch2Min = 1506, ch2Middle = 1509, ch2Max = 1506;	//max is knob up
		int ch3Min = 1506, ch3Max = 1506;	//max is knob up
		int ch4Min = 1506, ch4Middle = 1506, ch4Max = 1506;	//max is knob right

		Buzzer.on(600);
		do // calibrate min and max for each knob
		{
			OrientationSensor.update_current_RPY();	//ahrs calculation has to be run repetedly

			if (!update_channels())	//lost communication with TX
			{
				Buzzer.off();
				return;
			}
			
			ch1Min = ch1Min < CH1 ? ch1Min : CH1;
			ch2Min = ch2Min < CH2 ? ch2Min : CH2;
			ch3Min = ch3Min < CH3 ? ch3Min : CH3;
			ch4Min = ch4Min < CH4 ? ch4Min : CH4;

			ch1Max = ch1Max > CH1 ? ch1Max : CH1;
			ch2Max = ch2Max > CH2 ? ch2Max : CH2;
			ch3Max = ch3Max > CH3 ? ch3Max : CH3;
			ch4Max = ch4Max > CH4 ? ch4Max : CH4;
		} while (CH5 > ch5Min + 10); // turn CH5 left to finish

		Buzzer.off();

		Buzzer.on(950);
		do // calibrate channels middle value (when knobs are not touched) 
		{
			OrientationSensor.update_current_RPY();	 //ahrs calculation has to be run repetedly
			if (!update_channels())	//if lost communication with TX
			{
				Buzzer.off();
				return;
			}
			
			ch1Middle = CH1;
			ch2Middle = CH2;
			ch4Middle = CH4;
		} while (CH6 > ch6Min + 10);

		// save calibrated values to RAM and EEPROM
#define save(channel_parameter) RCReceiverClass::channel_parameter = channel_parameter;\
								EEPROM_variables.save(static_cast<int>(EEPROM_variables.channel_parameter), static_cast<float>(channel_parameter));
		save(ch1Min);
		save(ch2Min);
		save(ch3Min);
		save(ch4Min);
		save(ch1Max);
		save(ch2Max);
		save(ch3Max);
		save(ch4Max);
		save(ch1Middle);
		save(ch2Middle);
		save(ch4Middle);
#undef save()

		RCReceiverClass::ThrottleMin = ch3Min + ThrottleBuffer;
		RCReceiverClass::ThrottleMax = ch3Max;

		calibrated = true;

		Buzzer.off();
	}
}


RCReceiverClass RCReceiver;
