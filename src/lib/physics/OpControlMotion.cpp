#include "lib/physics/OpControlMotion.h"
#include "Controller.h"
#include "Odometry.h"
#include <cmath>
#include <cstdio>

Motion::MotorVoltages OpControlMotion::calculateVoltages(kinState state) {
	int power = sController->getAnalog(Controller::left_y);
	int turn = sController->getAnalog(Controller::right_x);

	double left = ((power + turn) / 127.0) * 12000.0;
	double right = ((power - turn) / 127.0) * 12000.0;

	static int count;
	double speed = std::sqrt(std::pow(sOdom->getCurrentState().velocity().x, 2) + std::pow(sOdom->getCurrentState().velocity().y, 2));

	if (speed != 0){
		count++;
		printf("%.i %.2f \n", count, speed);
	}

	return {left, right};
}

bool OpControlMotion::isSettled(kinState state) {
	return false;
}