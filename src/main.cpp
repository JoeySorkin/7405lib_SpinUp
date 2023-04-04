#include "main.h"
#include "Drive.h"
#include "Logger.h"
#include "Robot.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/kinState.h"
#include "lib/physics/PIDMotion.h"
#include "lib/physics/PIDTurn.h"
#include "lib/physics/TimedMotion.h"
#include "pros/rtos.hpp"
#include <cstdio>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

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
void disabled() {}

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
void autonomous() {}

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
	LoggerPtr logger = sLogger->createSource("OpControl");
	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(90, PID(1.5, 0.01, 0.2, true, 10)));
	// sDrive->waitUntilSettled();
	// pros::lcd::print(6, "Movement done");

	// with heading correction
	sDrive->setCurrentMotion(std::make_unique<PIDMotion>(Pose(0, 12), PID(1.5, 0.01, 0.2, true, 10),
	                                                     PID(1.5, 0.01, 0.2, true, 10), 0.5));
	if (sDrive->waitUntilSettled()) {
		logger->info("Done with movement\n");
	} else {
		logger->warning("Movement timed out\n");
	}
	pros::lcd::print(6, "Movement done");


	int counter = 1;
	// while (true) {
	// 	// 	// printf("before calling logger->error\n");
	// 	// 	// logger->info("Hello world! {}\n", counter++);
	// 	logger->error("Hello world! {}\n", counter++);
	// 	// 	// printf("%d\n", counter++);
	// 	// 	// pros::lcd::print(6, "%d", counter++);
	// 	pros::delay(20);
	// }
}
