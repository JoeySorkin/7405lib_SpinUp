#pragma once
#include "Motion.h"

class OpControlMotion : public Motion {
public:
	MotorVoltages calculateVoltages(kinState state);
	bool isSettled(kinState state);
};