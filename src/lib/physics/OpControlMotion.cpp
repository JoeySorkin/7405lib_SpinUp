#include "lib/physics/OpControlMotion.h"
#include "Controller.h"
#include "lib/utils/Math.h"

Motion::MotorVoltages OpControlMotion::calculateVoltages(kinState state) {
	int power = sController->getAnalog(Controller::left_y);
	int turn = sController->getAnalog(Controller::right_x);

	double a = 0;
	double b = 1;
	int curved_turn = (util::sign(turn / 127.0) * (a + (b - a) * (std::pow(fabs(turn / 127.0), 1.37)))) * 127.0;


	double left = (power + turn) / 127.0 * 12000;
	double right = (power - turn) / 127.0 * 12000;

	return {left, right};
}

bool OpControlMotion::isSettled(kinState state) {
	return false;
}