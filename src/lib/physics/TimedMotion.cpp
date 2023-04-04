//
// Created by Joey Sorkin on 3/29/23.
//

#include "lib/physics/TimedMotion.h"


Motion::MotorVoltages TimedMotion::calculateVoltages(kinState state) {
	return {2000.0, 2000.0};
}

bool TimedMotion::isSettled(kinState state) {
	return false;
}