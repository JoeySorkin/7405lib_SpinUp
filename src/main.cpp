#include "main.h"
#include "Drive.h"
#include "Logger.h"
#include "Odometry.h"
#include "Robot.h"
#include "lib/controllers/PID.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/kinState.h"
#include "lib/physics/NullMotion.h"
#include "lib/physics/OpControlMotion.h"
#include "lib/physics/PIDMotion.h"
#include "lib/physics/PIDTurn.h"
#include "lib/physics/ProfiledMotion.h"
#include "lib/physics/TimedMotion.h"
#include "pros/rtos.hpp"
#include <cstdio>
#include <memory>
#include "Shooter.h"
#include "Intake.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	sRobot->initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	// During matches - saves file data after auton, and after opcontrol
	sLogger->flush();
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
	LoggerPtr logger = sLogger->createSource("AUTONOMOUS");

	while (true) {
		logger->info("AUTONOMOUS LOOP\n");
		pros::delay(100);
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	// sRobot->setOpMode(Robot::DRIVER);
	// sDrive->setCurrentMotion(std::make_unique<OpControlMotion>());
	sRobot->setOpMode(Robot::AUTONOMOUS);

	sIntake->moveVoltage(12000);
	// sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(sOdom->getCurrentState().position.distanceTo(Pose(0.0, 35.0))));
	// sDrive->waitUntilSettled(2000);
	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(sOdom->getCurrentState().position.headingToPoint(Pose(10.0, 10.0, 60.0)), PID(200.0, 0.0, 700.0)));
	
	LoggerPtr logger = sLogger->createSource("Profiled motion");
}
