#include "lib/physics/PIDTurn.h"
#include "lib/utils/Math.h"

PIDTurn::PIDTurn(double targetHeading, PID pid, double threshold)
    : targetHeading(targetHeading), pid(pid), counter(0), threshold(threshold) {}

Motion::MotorVoltages PIDTurn::calculateVoltages(kinState state) {
	double error = util::getShortestAngle(util::toDeg(state.position.getTheta()), targetHeading);

	if (error < threshold) {
		counter++;
	} else {
		counter = 0;
	}

	double turnPwr = pid(error);

	return {turnPwr, -turnPwr};
}

bool PIDTurn::isSettled(kinState state) {
	return counter > 5;
}