#pragma once
#include "Constants.h"
#include "Motion.h"
#include "lib/geometry/TrapezoidProfile.h"

class ProfiledMotion : public Motion {
private:
	double threshold;
	TrapezoidProfile profile;

public:
	ProfiledMotion(double dist, double maxVel = chassis::maxVel, double accel = chassis::accel,
	               double decel = chassis::deccel, double threshold = 0.5);
	MotorVoltages calculateVoltages(kinState state) override;
	bool isSettled(kinState state) override;
};