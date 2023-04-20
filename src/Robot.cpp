
#include "Robot.h"
#include "Controller.h"
#include "Drive.h"
#include "Flywheel.h"
#include "Intake.h"
#include "Logger.h"
Robot* Robot::INSTANCE = nullptr;

void Robot::initialize() {
	sLogger->initialize("test.txt");
//	sController->initialize();
	sOdom->initialize();
	sDrive->initialize();
	sFlywheel->initialize();
	sIntake->initialize();
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
