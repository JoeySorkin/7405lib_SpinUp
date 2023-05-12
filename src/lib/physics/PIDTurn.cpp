#include "lib/physics/PIDTurn.h"
#include "Drive.h"
#include "lib/utils/Math.h"
#include "pros/motors.h"

LoggerPtr PIDTurn::logger = sLogger.createSource("PIDTurn", 0);

PIDTurn::PIDTurn(double targetHeading, PID pid, bool brakeLeft, bool brakeRight, double threshold, double degree)
    : targetHeading(targetHeading), pid(pid), counter(0), threshold(std::pow(threshold, 1.0 / degree)),
      brakeLeft(brakeLeft), brakeRight(brakeRight), degree(degree) {}

Motion::MotorVoltages PIDTurn::calculateVoltages(kinState state) {
	double error_true = util::getShortestAngle(util::toDeg(state.position.getTheta()), targetHeading);
	//	double error_true = targetHeading - state.position.getShortTheta();
	double error = util::sign(error_true) * std::pow(fabs(error_true), degree);

	pros::motor_brake_mode_e prevBrakeMode = sDrive.getBrakeMode();

	if (brakeLeft) { sDrive.setBrakeModeLeft(pros::E_MOTOR_BRAKE_HOLD); }
	if (brakeRight) { sDrive.setBrakeModeRight(pros::E_MOTOR_BRAKE_HOLD); }

	if (std::abs(error_true) < threshold) {
		counter++;
	} else {
		counter = 0;
	}

	double turnPwr = pid(error);


	double leftPwr = brakeLeft ? 0 : turnPwr;
	double rightPwr = brakeRight ? 0 : -turnPwr;
	if ((!brakeLeft)) {
		leftPwr = util::sign(leftPwr) * util::lerp(1400, 12000.0, util::clamp(0.0, 1.0, fabs(leftPwr / 12000.0)));
	}
	if ((!brakeRight)) {
		rightPwr = util::sign(rightPwr) * util::lerp(1400, 12000.0, util::clamp(0.0, 1.0, fabs(rightPwr / 12000.0)));
	}
	logger->debug("Error: {:.2f} Counter: {} Left Pwr: {} Right Pwr: {}\n", error_true, counter, leftPwr, rightPwr);

	return {leftPwr, rightPwr};
}

bool PIDTurn::isSettled(kinState state) {
	return counter > 5;
}