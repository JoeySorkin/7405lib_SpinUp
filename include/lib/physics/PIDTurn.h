#pragma once
#include "Motion.h"
#include "lib/controllers/PID.h"

// this entire motion just uses everything in degrees cause its easier
class PIDTurn : public Motion {
private:
	double targetHeading;
	double threshold;
	PID pid;
	int counter;// counter for how long we've been at a pos

public:
	PIDTurn(double targetHeading, PID pid, double threshold = 0.5);
	MotorVoltages calculateVoltages(kinState state) override;
	bool isSettled(kinState state) override;
};