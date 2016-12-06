// 
// 
// 

#include "OrientationSensor.h"
#include "Buzzer.h"
#include "Wire\Wire.h"
#include "OrientationSensor2.h"


template <typename T>
static void swap(T& first, T& second)
{
	auto temp = first;
	first = second;
	second = temp;
};

bool OrientationSensorClass::setup()
{
	OrientationSensor2::setup(); return true;

	Wire.begin();

	ret = mympu_open(200);
	//Serial.begin(115200);
	Serial.print("MPU init: "); Serial.println(ret);
	return true;
	// TODO calibration routine
}

bool OrientationSensorClass::update_current_RPY()
{
	float ROLL, PITCH, YAW, YAWRATE;
	OrientationSensor2::get_RPY(ROLL, PITCH, YAW, YAWRATE);

	currentRPY->setRPY(ROLL, PITCH, YAW, YAWRATE); 
	return true;

	static unsigned int c = 0; //cumulative number of successful MPU/DMP reads
	static unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
	static unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
	static unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

	ret = mympu_update();

	switch (ret)
	{
	case 0: c++; break;
	case 1: np++; return false;
	case 2: err_o++; return false;
	case 3: err_c++; return false;
	default:
	{
		static auto t = millis();
		if (t + 100 < millis())
		{
			t = millis();
			Buzzer.toggle();
		}
	}
	Serial.print("READ ERROR!  ");
	Serial.println(ret);
	return false;
	}

	if (ret == 0)
	{
		float roll = mympu.ypr[2];
		if (roll < -179.89f || roll > 179.89f)		//to fix roll jumping between 180 and -180

			roll = 0;

		else
			roll = roll > 0 ? roll - 180.0 : roll + 180.0;
		float pitch = mympu.ypr[1];
		float yaw = mympu.ypr[0];
		float yawRate = mympu.gyro[0];
		pitch -= pitch_OFFSET;
		roll -= roll_OFFSET;

		swap(roll, pitch);	// depends on Base Frame
		roll *= -1.0;	// depends on Base Frame
		pitch *= -1.0;	// depends on Base Frame
		yawRate *= -1.0;	// depends on Base Frame

		currentRPY->setRPY(roll, pitch, yaw, yawRate);
	}
	return true;
}


OrientationSensorClass OrientationSensor;