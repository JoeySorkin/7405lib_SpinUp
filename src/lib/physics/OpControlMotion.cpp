#include "lib/physics/OpControlMotion.h"
#include "Controller.h"
#include <cstdio>

Motion::MotorVoltages OpControlMotion::calculateVoltages(kinState state) {
	int power = sController->getAnalog(Controller::left_y);
	int turn = sController->getAnalog(Controller::right_x);

	double left = (power + turn) / 127.0 * 12000;
	double right = (power - turn) / 127.0 * 12000;
	printf("left_y: %.i \n",sController->getAnalog(Controller::left_x));
	printf("right_x: %.i \n",sController->getAnalog(Controller::right_x));

	return {left, right};
}

bool OpControlMotion::isSettled(kinState state) {
	return false;
}