//
// Created by Joey Sorkin on 3/29/23.
//

#include "lib/physics/TimedMotion.h"
#include "pros/rtos.hpp"

// idk if this is the implementation that you guys were planning on
// but this is what i did based on deduction from what the current code was present
TimedMotion::TimedMotion(uint32_t time, double power) : delay(time), power(power) {}

Motion::MotorVoltages TimedMotion::calculateVoltages(kinState state) {
	return {power, power};
}

bool TimedMotion::isSettled(kinState state) {
	return pros::millis() >= startTime + delay;
}