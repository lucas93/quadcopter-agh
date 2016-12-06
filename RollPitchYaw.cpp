#include "RollPitchYaw.h"
RollPitchYaw::RollPitchYaw()
{
	roll = 0.0;
	pitch = 0.0;
	yaw = 0.0;
	yawRate = 0.0;
}

RollPitchYaw::RollPitchYaw(const RollPitchYaw& other)
{
	roll = other.roll;
	pitch = other.pitch;
	yaw = other.yaw;
	yawRate = other.yawRate;
}

RollPitchYaw::RollPitchYaw(float roll, float pitch, float yaw, float yawRate) : roll(roll), pitch(pitch), yaw(yaw), yawRate(yawRate)
{
}


RollPitchYaw& RollPitchYaw::operator =(RollPitchYaw& other)
{
	roll = other.roll;
	pitch = other.pitch;
	yaw = other.yaw;
	yawRate = other.yawRate;
	return *this;
}


RollPitchYaw RollPitchYaw::operator -(RollPitchYaw& other)
{
	RollPitchYaw result(roll - other.roll, pitch - other.pitch, yaw - other.yaw, yawRate - other.yawRate);
	return result;
}

void RollPitchYaw::setRPY(const float roll, const float pitch, const float yaw, const float yawRate)
{
	this->roll = roll;
	this->pitch = pitch;
	this->yaw = yaw;
	this->yawRate = yawRate;
}


RollPitchYaw* RollPitchYaw::get_current_RPY_pointer()
{
	static RollPitchYaw currentRPY;
	return &currentRPY;
}


RollPitchYaw* RollPitchYaw::get_target_RPY_pointer()
{
	static RollPitchYaw targetRPY;
	return &targetRPY;
}






