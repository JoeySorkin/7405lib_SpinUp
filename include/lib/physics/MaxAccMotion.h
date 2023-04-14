#pragma once
#include "Motion.h"

// this entire motion just uses everything in degrees cause its easier
class MaxAccMotion : public Motion {
public:
	MotorVoltages calculateVoltages(kinState state) override;
	bool isSettled(kinState state) override;
};