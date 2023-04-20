#include "main.h"
#include "Drive.h"
#include "Flywheel.h"
#include "Intake.h"
#include "Logger.h"
#include "Robot.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/kinState.h"
#include "lib/physics/NullMotion.h"
#include "lib/physics/OpControlMotion.h"
#include "lib/physics/PIDMotion.h"
#include "lib/physics/PIDTurn.h"
#include "lib/physics/ProfiledMotion.h"
#include "lib/physics/TimedMotion.h"
#include "lib/utils/Math.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cstdio>

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
	sDrive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	sDrive->setCurrentMotion(std::make_unique<OpControlMotion>());
	LoggerPtr logger = sLogger->createSource("OpControl");


	while(true){
		pros::delay(20);
	}
	sOdom->reset();


	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(90, PID(1.5, 0.01, 0.2, true, 10)));

	// with heading correction
	// sDrive->setCurrentMotion(std::make_unique<PIDMotion>(Pose(0, 12), PID(1.5, 0.01, 0.2, true, 10),
	//                                                      PID(1.5, 0.01, 0.2, true, 10), 0.5));

	// sDrive->setCurrentMotion(std::make_unique<TimedMotion>(750, 4000));
	// if (sDrive->waitUntilSettled(4000)) {
	// 	logger->info("Done with movement\n");
	// } else {
	// 	logger->warn("Movement timed out\n");
	// }
	// sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	// pros::lcd::print(6, "Movement done");


	// tuning profiled motions
	sDrive->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	// sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(70, 50, 60, 60, 0.1));
	// sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(70, 20, 60, 60, 0.1));
	// sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(50, 50, 60, 60, 0.1));
	// sDrive->waitUntilSettled(10000);
	// sDrive->setCurrentMotion(std::make_unique<NullMotion>());

	// start of auton
//
//	sDrive->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
//	sIntake->moveVoltage(12000);
//
//	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(22.91, 50, 60, 60, 0.1));
//	sDrive->waitUntilSettled(2500);
//	sIntake->moveVoltage(0);
//
//	pros::delay(2000);
//
//	sDrive->setCurrentMotion(std::make_unique<NullMotion>());


	sFlywheel->setVelocity(2500);

		sIntake->moveVoltage(12000);

		sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(22.91, 50, 60, 60, 0.1));
		sDrive->waitUntilSettled(2500);
		sIntake->moveVoltage(0);

		pros::delay(2000);



	sFlywheel->waitUntilSettled();

	Pose goal = Pose(42,109, 0);
	double turn_amount = sOdom->getCurrentState().position.headingToPoint(goal) * 180.0/M_PI;

	pros::lcd::print(0, "turn amount %.2f", turn_amount);
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turn_amount, PID(500,0.1,0.1)));
	sDrive->waitUntilSettled(5000);
	pros::delay(2000);
	sFlywheel->triple_shoot();
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
//	{H: 29.85, 0.053221, 22.8943216}
//	{H: 48.7249, 0.703349, 70.381138}

}
