#include <Wire.h>
#include "SerialInputProcesser.h"
#include "Controller.h"
#include "RCtoRPY.h"
#include <Servo.h>
#include "RCReceiver.h"
#include "Motors.h"
#include "Buzzer.h"
#include "Led.h"
#include "Motors.h"
#include "RollPitchYaw.h"
#include "PID.h"
#include "EEPROM_variables.h"
#include "OrientationSensor.h"

#define MOTORS_PINS 20, 21, 22, 23
#define LED_PIN 13

const bool DEBUG = true;
const bool DEBUG_RC = false;
const bool DEBUG_RPY_CURRENT = false;
const bool DEBUG_RPY_TARGET = false;
const bool DEBUG_IS_ARMED = false;
#define DEBUG_REFRESH_TIME  50


const int T_ms = 5;	// cycle time in [ms], T==5 -> 200Hz

void setup()
{
	Serial.begin(128000);
	Wire.begin();
#if ARDUINO >= 157
		Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#endif
	Led.setup(LED_PIN);

	if (!OrientationSensor.setup())	//check if IMU is working correctly
	{
		digitalWrite(13, true);
		while (true)
		{
			Serial.println("MPU not working!!");
			Buzzer.buzzSignal("ooo");
		}
	}
	SerialInputProcesser.setup();
	RCReceiver.setup();
	RCtoRPY.setup();
	Controller.setup(T_ms);
	Motors.setup(MOTORS_PINS);

	Buzzer.buzzSignal("a");
}


void loop()
{
	static auto t = micros();
	t = micros();
	
	OrientationSensor.update_current_RPY();
	Led.blink_frequently(200);
	SerialInputProcesser.proccess_serial_if_any_and_unarmed();
	RCReceiver.update_channels();
	RCReceiver.calibrate_RC_if_ready_and_not_armed();
	RCtoRPY.update_target_RPY();	
	Controller.control_loop();

	if (DEBUG)
		debug();


	//print_frequency();
	
	 while (t + T_ms*1000 > micros());	//set refresh rate to 200Hz
}

void debug()
{
#ifdef DEBUG_REFRESH_TIME

	static auto t_debug = millis();
	if (t_debug + DEBUG_REFRESH_TIME < millis())
	{
		t_debug = millis();

#endif	// #ifdef DEBUG_REFRESH_TIME

		if (DEBUG_RC)
		{
			Serial.println(String(RCReceiver.CH1) +
				+"\t" + RCReceiver.CH2
				+ "\t" + RCReceiver.CH3
				+ "\t" + RCReceiver.CH4
				+ "\t" + RCReceiver.CH5
				+ "\t" + RCReceiver.CH6 + "\t");
		}

		if (DEBUG_RPY_TARGET)
		{
			String rollPitchYawString = String(RollPitchYaw::get_target_RPY_pointer()->roll, 2) + "\t" +
				String(RollPitchYaw::get_target_RPY_pointer()->pitch, 2) + "\t" +
				String(RollPitchYaw::get_target_RPY_pointer()->yaw, 2) + "\t" +
				String(RollPitchYaw::get_target_RPY_pointer()->rollRate, 2) + "\t" +
				String(RollPitchYaw::get_target_RPY_pointer()->pitchRate, 2) + "\t" +
				String(RollPitchYaw::get_target_RPY_pointer()->yawRate, 2) + "\t";
			Serial.println(rollPitchYawString);
		}

		if (DEBUG_RPY_CURRENT)
		{
			String rollPitchYawString = String(RollPitchYaw::get_current_RPY_pointer()->roll, 2) + "\t" +
				String(RollPitchYaw::get_current_RPY_pointer()->pitch, 2) + "\t" +
				String(RollPitchYaw::get_current_RPY_pointer()->yaw, 2) + "\t" +
				String(RollPitchYaw::get_current_RPY_pointer()->rollRate, 2) + "\t" +
				String(RollPitchYaw::get_current_RPY_pointer()->pitchRate, 2) + "\t" +
				String(RollPitchYaw::get_current_RPY_pointer()->yawRate, 2) + "\t";
			Serial.println(rollPitchYawString);
		}

		if (DEBUG_IS_ARMED)
		{				
			Serial.println(*(Motors.get_arm_indicator_pointer()) == true ? "ARMED \t" : "NOT ARMED \t");			
		}


#ifdef DEBUG_REFRESH_TIME

	}

#endif	// #ifdef DEBUG_REFRESH_TIME
}

	void print_frequency()
	{
		static auto t = micros();
		static auto dt = micros() - t;
		dt = micros() - t;
		t = micros();
		Serial.println(String("\tFrequency: ") + 1000000.0 / double(dt) + " Hz");
	}