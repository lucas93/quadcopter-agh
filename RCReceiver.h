// RCReceiver.h

#ifndef _RCRECEIVER_h
#define _RCRECEIVER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "EEPROM_variables.h"

#include "Motors.h"

class RCReceiverClass
{
private:
	
	// values for all the channels
	volatile uint16_t ch6_start = 0, ch6_global_count = 0;
	volatile uint16_t ch5_start = 0, ch5_global_count = 0;
	volatile uint16_t ch4_start = 0, ch4_global_count = 0;
	volatile uint16_t ch3_start = 0, ch3_global_count = 0;
	volatile uint16_t ch2_start = 0, ch2_global_count = 0;
	volatile uint16_t ch1_start = 0, ch1_global_count = 0;

	volatile uint8_t flag = LOW;//global flag

	volatile uint8_t ch6_global_flag = LOW;//global flag
	volatile uint8_t ch5_global_flag = LOW;//global flag
	volatile uint8_t ch4_global_flag = LOW;//global flag
	volatile uint8_t ch3_global_flag = LOW;//global flag
	volatile uint8_t ch2_global_flag = LOW;//global flag
	volatile uint8_t ch1_global_flag = LOW;//global flag

	

	const int ThrottleBuffer = 100;							//throttle buffer above ch3Min to start engines
	int ThrottleMin = ch3Min + ThrottleBuffer;			//minimal throttle to start engines
	int ThrottleMax = ch3Max;

	static void ch6_count();
	static void ch5_count();
	static void ch4_count();
	static void ch3_count();
	static void ch2_count();
	static void ch1_count();

	bool* RC_is_connected;		//for lost communication, indicates if RC is connected to quadcopter
	bool* motors_is_armed;

public:
	int const ch6_pin = 6;
	int const ch5_pin = 5;
	int const ch4_pin = 4;
	int const ch3_pin = 3;
	int const ch2_pin = 2;
	int const ch1_pin = 1;

	volatile uint16_t CH1 = 1;	//current RC channels values
	volatile uint16_t CH2 = 1;	//current RC channels values
	volatile uint16_t CH3 = 1;	//current RC channels values
	volatile uint16_t CH4 = 1;	//current RC channels values
	volatile uint16_t CH5 = 1;	//current RC channels values
	volatile uint16_t CH6 = 1;	//current RC channels values

	// RC values range
	int ch1Min = 1140, ch1Middle = 1506, ch1Max = 1901;	// TODO: channel discription
	int ch2Min = 1166, ch2Middle = 1509, ch2Max = 1912;	// TODO: channel discription
	int ch3Min = 1119, ch3Max = 1888;					// TODO: channel discription
	int ch4Min = 1123, ch4Middle = 1530, ch4Max = 1895;	// TODO: channel discription
	int const ch5Min = 1009, ch5Max = 2034;
	int const ch6Min = 1009, ch6Max = 2034;

	void setup();
	bool* get_RC_connection_indicator_pointer();
	int get_ThrottleMin();
	bool update_channels();	// returns false when connection is lost
	void calibrate_RC_if_ready_and_not_armed();
};


extern RCReceiverClass RCReceiver;

#endif

