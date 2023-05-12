#include "main.h"
#include "AutonSelector.h"
#include "Drive.h"
#include "Intake.h"
#include "Logger.h"
#include "Robot.h"
#include "autons.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/kinState.h"
#include "lib/physics/NullMotion.h"
#include "lib/physics/OpControlMotion.h"
#include "lib/physics/PIDMotion.h"
#include "lib/physics/PIDTurn.h"
#include "lib/physics/ProfiledMotion.h"
#include "lib/physics/ProfiledTurn.h"
#include "lib/physics/TimedMotion.h"
#include "lib/utils/Math.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cstdio>
#include <ios>

AutonSelector autonSelector;
void initialize() {
	// TODO: slight rewrite of auton selector
	autonSelector.initialize();
	sRobot.initialize();
	autonSelector.showSelector();
	autonSelector.restoreLCD();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	// During matches - saves file data after auton, and after opcontrol
	sLogger.flush();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	switch (sRobot.getAuton()) {
		case Auton::LEFT:
			leftAuton();
			break;
		case Auton::RIGHT:
			rightAuton();
			break;
		default:
			break;
	}
}

void opcontrol() {
	sDrive.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	sDrive.setCurrentMotion(std::make_unique<OpControlMotion>());
}
