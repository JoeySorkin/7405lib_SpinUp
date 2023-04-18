#pragma once
#include "Constants.h"
#include "Logger.h"
#include "Motion.h"
#include "lib/controllers/PID.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/TrapezoidProfile.h"

class ProfiledMotion : public Motion {
private:
	double threshold;
	double errorSum;
	Pose startPose;
	TrapezoidProfile profile;
	int prevSign;

	static LoggerPtr logger;

public:
	ProfiledMotion(double dist, double maxVel = chassis::maxVel, double accel = chassis::accel,
	               double decel = chassis::deccel, double threshold = 0.5);
	void start() override;
	MotorVoltages calculateVoltages(kinState state) override;
	bool isSettled(kinState state) override;
};