// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Kalman.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
namespace OrientationSensor2
{
	// class default I2C address is 0x68
	// specific I2C addresses may be passed as a parameter here
	// AD0 low = 0x68 (default for InvenSense evaluation board)
	// AD0 high = 0x69
	class MPU6050_modified : public MPU6050
	{
	private:
		int16_t ax_offset = 0;
		int16_t ay_offset = 0;
		int16_t az_offset = 0;
		int16_t gx_offset = 0;
		int16_t gy_offset = 0;
		int16_t gz_offset = 0;
	public:
		void setXAccelOffset(int16_t offset) { ax_offset = offset; }
		void setYAccelOffset(int16_t offset) { ay_offset = offset; }
		void setZAccelOffset(int16_t offset) { az_offset = offset; }
		void setXGyroOffset(int16_t offset) { gx_offset = offset; }
		void setYGyroOffset(int16_t offset) { gy_offset = offset; }
		void setZGyroOffset(int16_t offset) { gz_offset = offset; }
		void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
		{
			MPU6050::getMotion6(ax, ay, az, gx, gy, gz);
#define int16_t_boundry(param) *param = (int32_t(*param) - param##_offset < -32768 ? -32768 : (int32_t(*param) - param##_offset > 32767 ? 32767 : *param - param##_offset))

			int16_t_boundry(ax); int16_t_boundry(ay); int16_t_boundry(az);
			int16_t_boundry(gx); int16_t_boundry(gy); int16_t_boundry(gz);
#undef int16_t_boundry
		}
		void getMotion6_metric_units(float* ax, float* ay, float* az, float* gx, float* gy, float* gz)
		{
			int16_t ax_, ay_, az_, gx_, gy_, gz_;
			getMotion6(&ax_, &ay_, &az_, &gx_, &gy_, &gz_);
			*ax = static_cast<float>(ax_) / 16384.f * 9.8105f;	// int16_t to [m/s^2]
			*ay = static_cast<float>(ay_) / 16384.f * 9.8105f;	// int16_t to [m/s^2]
			*az = static_cast<float>(az_) / 16384.f * 9.8105f;	// int16_t to [m/s^2]
			*gx = static_cast<float>(gx_) / 131.072f;	// int16_t to [deg/s]
			*gy = static_cast<float>(gy_) / 131.072f;	// int16_t to [deg/s]
			*gz = static_cast<float>(gz_) / 131.072f;	// int16_t to [deg/s]
		}
	};

	MPU6050_modified accelgyro;
	Kalman kalmanX; // Create the Kalman instances
	Kalman kalmanY;
	double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
	decltype(micros()) timer;
	//MPU6050 accelgyro(0x69); // <-- use for AD0 high

	int16_t ax, ay, az;
	int16_t gx, gy, gz;
#define accX ax
#define accY ay
#define accZ az
#define gyroX gx
#define gyroY gy
#define gyroZ gz

	// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
	// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
	// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
	bool blinkState = false;

	void setup() {
		// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		//Wire.begin();
#if ARDUINO >= 157
		//Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#endif // ARDUINO >= 157
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
#endif // I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE

		// initialize serial communication
		// (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
		// it's really up to you depending on your project)
		//Serial.begin(115200);

		//for (size_t i = 5; i > 0; i--)
		//{
		//	Serial.println(i);
		//	delay(1000);
		//}

		// initialize device
		Serial.println("Initializing I2C devices...");
		accelgyro.initialize();

		// verify connection
		Serial.println("Testing device connections...");
		Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

		accelgyro.setXAccelOffset(431);
		accelgyro.setYAccelOffset(237);
		accelgyro.setZAccelOffset(0);
		accelgyro.setXGyroOffset(-87);
		accelgyro.setYGyroOffset(30);
		accelgyro.setZGyroOffset(-113);

		accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


#ifdef RESTRICT_PITCH // Eq. 25 and 26
		double roll = atan2(accY, accZ) * RAD_TO_DEG;
		double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
		double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
		double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif // RESTRICT_PITCH

		kalmanX.setAngle(roll); // Set starting angle
		kalmanY.setAngle(pitch);
		timer = micros();

		// configure Arduino LED for
		pinMode(LED_PIN, OUTPUT);
	}

	void get_RPY(float& ROLL, float& PITCH, float& YAW, float& YAWRATE)
	{
		auto t = millis();
		// read raw accel/gyro measurements from device
		accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

		double dt = (double)(micros() - timer) / 1000000.0; // Calculate delta time
		timer = micros();

#ifdef RESTRICT_PITCH // Eq. 25 and 26
		double roll = atan2(accY, accZ) * RAD_TO_DEG;
		double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
		double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
		double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
#define int16_t_to_rad_per_s (1.0 / 131.072f)
		double gyroXrate = static_cast<double>(gyroX) * int16_t_to_rad_per_s; // Convert to deg/s
		double gyroYrate = static_cast<double>(gyroY) * int16_t_to_rad_per_s; // Convert to deg/s
		double gyroZrate = static_cast<double>(gyroZ) * int16_t_to_rad_per_s; // Convert to deg/s

#ifdef RESTRICT_PITCH
																		  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
			kalmanX.setAngle(roll);
			kalAngleX = roll;
		}
		else
			kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleX) > 90)
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
		kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
																		  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
			kalmanY.setAngle(pitch);
			kalAngleY = pitch;
		}
		else
			kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleY) > 90)
			gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

		//Serial.println(String(kalAngleX, 4) + "\t" + String(kalAngleY, 4) + "\t" + gyroZ);

		//#ifdef OUTPUT_READABLE_ACCELGYRO
		//		// display tab-separated accel/gyro x/y/z values
		//	Serial.print("a/g:\t");
		//	Serial.print(ax); Serial.print("\t");
		//	Serial.print(ay); Serial.print("\t");
		//	Serial.print(az); Serial.print("\t");
		//	Serial.print(gx); Serial.print("\t");
		//	Serial.print(gy); Serial.print("\t");
		//	Serial.println(gz);
		//#endif

		// blink LED to indicate activity
		blinkState = !blinkState;
		digitalWrite(LED_PIN, blinkState);

		//print_frequency();
		//while (t + 2 > millis());
		ROLL = kalAngleY;
		PITCH = kalAngleX;
		YAW = 0.0f;
		YAWRATE = gyroZrate;
	}

	void print_frequency()
	{
		static auto t = micros();
		static auto dt = micros() - t;
		dt = micros() - t;
		t = micros();
		Serial.println(String("\tFrequency: ") + 1000000.0 / double(dt) + " Hz");
	}
};