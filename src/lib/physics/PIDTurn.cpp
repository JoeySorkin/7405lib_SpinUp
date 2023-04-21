#include "lib/physics/PIDTurn.h"
#include "Drive.h"
#include "lib/utils/Math.h"
#include "pros/motors.h"

LoggerPtr PIDTurn::logger = sLogger->createSource("PIDTurn", 0);

PIDTurn::PIDTurn(double targetHeading, PID pid, bool brakeLeft, bool brakeRight, double threshold)
    : targetHeading(targetHeading), pid(pid), counter(0), threshold(threshold), brakeLeft(brakeLeft),
      brakeRight(brakeRight) {}

Motion::MotorVoltages PIDTurn::calculateVoltages(kinState state) {
	double error = util::getShortestAngle(util::toDeg(state.position.getTheta()), targetHeading);

	pros::motor_brake_mode_e prevBrakeMode = sDrive->getBrakeMode();

	if (brakeLeft) { sDrive->setBrakeModeLeft(pros::E_MOTOR_BRAKE_HOLD); }
	if (brakeRight) { sDrive->setBrakeModeRight(pros::E_MOTOR_BRAKE_HOLD); }

	if (std::abs(error) < threshold) {
		counter++;
	} else {
		counter = 0;
	}

	double turnPwr = pid(error);

	logger->debug("Error: {:.2f} Counter: {} Pwr: {}\n", error, counter, turnPwr);

	double leftPwr = brakeLeft ? 0 : turnPwr;
	double rightPwr = brakeRight ? 0 : -turnPwr;

	return {leftPwr, rightPwr};
}

bool PIDTurn::isSettled(kinState state) {
	return counter > 5;
}