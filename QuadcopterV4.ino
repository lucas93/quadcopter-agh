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

#define DEBUG_RC false
#define DEBUG_RPY_CURRENT false
#define DEBUG_RPY_TARGET false
#define DEBUG_IS_ARMED false


const int T_ms = 2;	// cycle time in [ms], T==5 -> 200Hz

void setup()
{
	Serial.begin(128000);
	//Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
	Wire.begin();
#if ARDUINO >= 157
		Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#endif
	Led.setup(LED_PIN);

	if (!OrientationSensor.setup())	//check if IMU is working
	{
		for (int i = 0; i < 10; i++)
		{
			Serial.println(i);
			delay(1000);
		}
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
	while (!OrientationSensor.update_current_RPY());	// IMU update rate is 200Hz (T=5ms)
	Led.blink_frequently(200);
	SerialInputProcesser.proccess_serial_if_any_and_unarmed();
	RCReceiver.update_channels();
	RCReceiver.calibrate_RC_if_ready_and_not_armed();

	RCtoRPY.update_target_RPY();
	
	Controller.control_loop();

	debug();

	//print_frequency();

	 while (t + T_ms*1000 > micros());	//set refresh rate to 200Hz
}

void debug()
{
#if DEBUG_RC
	static auto t1 = millis();
	if (t1 + 100 < millis())
	{
		t1 = millis();
		Serial.println(String(RCReceiver.CH1) +
			+"\t" + RCReceiver.CH2
			+ "\t" + RCReceiver.CH3
			+ "\t" + RCReceiver.CH4
			+ "\t" + RCReceiver.CH5
			+ "\t" + RCReceiver.CH6 + "\t");
	}

#endif	// DEBUG_RC

#if DEBUG_RPY_TARGET
	{
		String rollPitchYawString = String(RollPitchYaw::get_target_RPY_pointer()->roll, 2) + "\t" +
			String(RollPitchYaw::get_target_RPY_pointer()->pitch, 2) + "\t" +
			String(RollPitchYaw::get_target_RPY_pointer()->yawRate, 2) + "\t";
		Serial.println(rollPitchYawString);
	}
#endif	// DEBUG_RPY

#if DEBUG_RPY_CURRENT
	{
		String rollPitchYawString = String(RollPitchYaw::get_current_RPY_pointer()->roll, 2) + "\t" +
			String(RollPitchYaw::get_current_RPY_pointer()->pitch, 2) + "\t" +
			String(RollPitchYaw::get_current_RPY_pointer()->yawRate, 2) + "\t";
		Serial.println(rollPitchYawString);
	}
#endif	// DEBUG_RPY

#if DEBUG_IS_ARMED
	{
		{
			static auto t2 = millis();
			if (t2 + 100 < millis())
			{
				t2 = millis();
				Serial.println(*(Motors.get_arm_indicator_pointer()) == true ? "ARMED \t" : "NOT ARMED \t");
			}
		}
#endif	// DEBUG_IS_ARMED
}

	void print_frequency()
	{
		static auto t = micros();
		static auto dt = micros() - t;
		dt = micros() - t;
		t = micros();
		Serial.println(String("\tFrequency: ") + 1000000.0 / double(dt) + " Hz");
	}