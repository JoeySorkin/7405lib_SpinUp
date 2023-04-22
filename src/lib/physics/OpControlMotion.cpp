#include "lib/physics/OpControlMotion.h"
#include "Controller.h"
#include "Odometry.h"
#include <cmath>
#include <cstdio>
#include "lib/utils/Math.h"
#include "math.h"

Motion::MotorVoltages OpControlMotion::calculateVoltages(kinState state) {
	int power = sController->getAnalog(Controller::left_y);
	int turn = sController->getAnalog(Controller::right_x);

	// eerp
	// double n = 2.5;
	// double turnOutput = (1.0/127.0) * std::abs(turn);
	// turnOutput = powf(turnOutput, n) * 12000 * util::sign(turn);

	// power = power/127.0 * 12000; 
	// turn = turnOutput + ((20.0 / 127.0) * 12000 * util::sign(turn));

	power = power/127.0 * 12000;
	turn = turn / 127.0 * 12000;

	// double speed = std::sqrt(std::pow(sOdom->getCurrentState().velocity().x, 2) + std::pow(sOdom->getCurrentState().velocity().y, 2));

	double left = (power + turn) ;
	double right = (power - turn);

	return {left, right};
}

bool OpControlMotion::isSettled(kinState state) {
	return false;
}