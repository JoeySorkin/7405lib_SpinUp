#include "lib/physics/PIDTurn.h"
#include "lib/utils/Math.h"
#include <cmath>
#include <cstdio>

PIDTurn::PIDTurn(double targetHeading, PID pid, double threshold)
    : targetHeading(targetHeading), pid(pid), counter(0), threshold(threshold) {}

Motion::MotorVoltages PIDTurn::calculateVoltages(kinState state) {
	double error = util::getShortestAngle(util::toDeg(state.position.getTheta()), targetHeading);
	printf("PIDTurn error: %.2f\n",  error);

	if (std::abs(error) < threshold) {
		counter++;
	} else {
		counter = 0;
	}

	

	double turnPwr = pid(error);

	turnPwr = util::sign(turnPwr) * ((std::fabs(turnPwr) > 12000) ? 12000 : std::fabs(turnPwr));

	return {turnPwr, -turnPwr};
}

bool PIDTurn::isSettled(kinState state) {
	return counter > 5;
}