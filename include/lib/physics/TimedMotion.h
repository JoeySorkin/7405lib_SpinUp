//
// Created by Joey Sorkin on 3/29/23.
//

#ifndef INC_7405SPINUP_TIMEDMOTION_H
#define INC_7405SPINUP_TIMEDMOTION_H

#include "Motion.h"
class TimedMotion : public Motion {
private:
	int power;
	uint32_t delay;

public:
	// time in ms, millivolts for both sides of drive to get
	TimedMotion(uint32_t time, int mv);
	MotorVoltages calculateVoltages(kinState state) override;
	bool isSettled(kinState state) override;
};

#endif// INC_7405SPINUP_TIMEDMOTION_H
