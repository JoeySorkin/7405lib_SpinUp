
#include "Robot.h"
#include "Drive.h"
#include "Logger.h"
Robot* Robot::INSTANCE = nullptr;

void Robot::initialize() {
	sOdom->initialize();
	sDrive->initialize();
	sLogger->initialize("test.txt");
	setOpMode(AUTONOMOUS);
}

void Robot::setOpMode(Robot::OpMode op) {
	opmode.store(op);
	switch (op) {
		case DRIVER:
			// sDrive->setBrakeMode(MOTOR_BRAKE_COAST);
			break;
		case AUTONOMOUS:
			// sDrive->setBrakeMode(MOTOR_BRAKE_HOLD);
			break;
	}
}

Robot::OpMode Robot::getOpMode() {
	return opmode.load();
}
