#pragma once
#include "Logger.h"
#include "Motion.h"
#include "lib/controllers/PID.h"

// this entire motion just uses everything in degrees cause its easier
class PIDTurn : public Motion {
private:
	double targetHeading;
	double threshold;
	double degree;
	PID pid;
	int counter;// counter for how long we've been at a pos

	bool brakeLeft;
	bool brakeRight;

	static LoggerPtr logger;


public:
	PIDTurn(double targetHeading, PID pid, bool brakeLeft = false, bool brakeRight = false, double threshold = 0.5, double degree = 1);
	MotorVoltages calculateVoltages(kinState state) override;
	bool isSettled(kinState state) override;
};
