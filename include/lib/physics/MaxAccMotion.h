#pragma once
#include "Motion.h"

// motion is for collecting test data of drive
// data printing is done in the drive
class MaxAccMotion : public Motion {
public:
	MotorVoltages calculateVoltages(kinState state) override;
	bool isSettled(kinState state) override;
};