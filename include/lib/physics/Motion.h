//
// Created by Joey Sorkin on 3/24/23.
//

#ifndef INC_7405SPINUP_MOTION_H
#define INC_7405SPINUP_MOTION_H
#include "../../main.h"
#include "lib/geometry/kinState.h"
class Motion {
protected:
	uint32_t start_time = 0;

public:
	struct MotorVoltages {
		double left, right;
	};
	Motion() = default;
	void start();
	virtual MotorVoltages calculateVoltages(kinState state) = 0;
	virtual bool isSettled(kinState state) = 0;
};

#endif// INC_7405SPINUP_MOTION_H
