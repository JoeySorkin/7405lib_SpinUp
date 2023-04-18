#include "lib/physics/MaxAccMotion.h"
#include "pros/rtos.hpp"

Motion::MotorVoltages MaxAccMotion::calculateVoltages(kinState state) {
	double time = (pros::millis() - startTime) / 1000.0;
	double pwr = time * -500;
	return {pwr, pwr};
	// return {-12000.0, -12000.0};
}

bool MaxAccMotion::isSettled(kinState state) {
	return false;
}