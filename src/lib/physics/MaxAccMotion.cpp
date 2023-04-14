#include "lib/physics/MaxAccMotion.h"

Motion::MotorVoltages MaxAccMotion::calculateVoltages(kinState state) {
	return {12000.0, 12000.0};
}

bool MaxAccMotion::isSettled(kinState state) {
	return false;
}