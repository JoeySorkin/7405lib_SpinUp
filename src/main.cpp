#include "main.h"
#include "AutonSelector.h"
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
#include "lib/physics/ProfiledTurn.h"
#include "lib/physics/TimedMotion.h"
#include "lib/utils/Math.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cstdio>

AutonSelector autonSelector;
void initialize() {
	autonSelector.initialize();
	sRobot->initialize();
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
void left_auton() {
	uint32_t startTime = pros::micros();
	LoggerPtr logger = sLogger->createSource("Left Auto");

	sDrive->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	// sFlywheel->setVelocity(2555);
	sFlywheel->setVelocity(2566);
	// sFlywheel->setVelocity(2710);// somehow we had to bump this up a lot
	sOdom->reset();

	// DONT TOUCH THIS POINT ANYMORE - IT FUCKS WITH THE AUTONS
	Pose goal = Pose(-8.75, 119.7, 0);
	// Pose goal = Pose(-5.3, 115.32, 0);

	// move back and hit roller
	logger->info("Moving back to hit roller\n");
	sDrive->setCurrentMotion(std::make_unique<TimedMotion>(1500, -7000));
	pros::delay(100);
	logger->info("Rolling roller\n");
	sIntake->moveVoltage(10000);
	pros::delay(300);
	sIntake->moveVoltage(0);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("Done rolling roller\n");

	logger->info("Arc Motion To Disk + Move to intake disc\n");
	// sDrive->setCurrentMotion(
	//         std::make_unique<PIDTurn>(-43, PID(70, 10, 0, true, 5), true, false, 0.5));// last minute fuckery

	sDrive->setCurrentMotion(
	        std::make_unique<PIDTurn>(-43, PID(120, 10, 30, true, 5), true, false, 0.5));// last minute fuckery

	sDrive->waitUntilSettled(1000);
	sIntake->moveVoltage(12000);
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(
	        Pose(-4.73, 7.41).distanceTo(sOdom->getCurrentState().position), 50, 60, 60, 0.3));
	sDrive->waitUntilSettled(2500);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("Intook disc, moving back\n");
	pros::delay(500);// change back to 50
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(
	        -1 * Pose(-1.84, 4.017).distanceTo(sOdom->getCurrentState().position), 50, 60, 60, 0.3));
	sDrive->waitUntilSettled(2500);
	sIntake->moveVoltage(0);


	double turnAmount = sOdom->getCurrentState().position.headingToPoint(goal) / M_PI * 180;
	logger->info("Rotate to basket to shoot. Heading to basket: {}\n", turnAmount);

	// sDrive->setCurrentMotion(
	//         std::make_unique<PIDTurn>(turnAmount, PID(6.3, 0.004, 4.8, true, 5), false, false, 0.5, 1.85));//
	//         original

	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turnAmount, PID(6.3, 0.003, 4.8, true, 5), false, false, 0.5,
	//                                                    1.85));// last minute fuckery

	// sDrive->setCurrentMotion(
	//         std::make_unique<PIDTurn>(turnAmount, PID(6.4, 0.00, 4.6, true, 5), false, false, 0.5, 1.67));// tuning

	sDrive->setCurrentMotion(
	        std::make_unique<PIDTurn>(turnAmount, PID(6.3, 0.004, 5.2, true, 5), false, false, 0.5, 1.73));// tuning

	sDrive->waitUntilSettled(1200);
	// sDrive->waitUntilSettled(2000);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("Turned to basket. Shooting discs\n");

	sFlywheel->waitUntilSettled();
	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(50);
	sIntake->moveVoltage(0);
	if (sFlywheel->waitUntilSettled(1250)) {
		logger->warn("Flywheel timed out\n");
	} else {
		logger->info("Flywheel stabilized\n");
	}
	// pros::delay(600);

	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(50);
	sIntake->moveVoltage(0);

	if (sFlywheel->waitUntilSettled(1250)) {
		logger->warn("Flywheel timed out\n");
	} else {
		logger->info("Flywheel stabilized\n");
	}

	sFlywheel->shoot();
	pros::delay(50);
	logger->info("Done shooting discs\n");
	sFlywheel->setVelocity(2430);

	logger->info("Rotating + Intaking 3 Stack\n");
	Pose threeStack = Pose(36.92, 34.726);
	turnAmount = sOdom->getCurrentState().position.headingToPoint(threeStack) / M_PI * 180;
	logger->info("Turn Amount to 3 stack: {}\n", turnAmount);
	// sDrive->setCurrentMotion(
	//         std::make_unique<PIDTurn>(turnAmount, PID(6.2, 0.002, 4.6, true, 5), false, false, 0.5, 1.55));

	// sDrive->setCurrentMotion(
	//         std::make_unique<PIDTurn>(turnAmount, PID(6.3, 0.0015, 4.6, true, 5), false, false, 0.5, 1.55));

	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turnAmount, PID(6.6, 0.002, 6, true, 5), false, false, 0.5,
	//                                                    1.9));// tune this maybe - keep watch

	sDrive->setCurrentMotion(
	        std::make_unique<PIDTurn>(turnAmount, PID(6.3, 0.004, 5.2, true, 5), false, false, 0.5, 1.7));// tuning

	//	sDrive->waitUntilSettled(1500);
	sDrive->waitUntilSettled(1300);
	sIntake->moveVoltage(12000);
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(threeStack.distanceTo(sOdom->getCurrentState().position),
	                                                          30, 60, 40, 0.25));
	sDrive->waitUntilSettled(2500);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());

	turnAmount = sOdom->getCurrentState().position.headingToPoint(goal) / M_PI * 180;

	// sDrive->setCurrentMotion(
	//         std::make_unique<PIDTurn>(turnAmount, PID(6.1, 0.003, 4.6, true, 5), false, false, 0.5, 1.7));

	// retune this last motion
	sDrive->setCurrentMotion(
	        std::make_unique<PIDTurn>(turnAmount, PID(6.3, 0.8, 5.2, true, 5), false, false, 0.5, 1.65));


	sDrive->waitUntilSettled(1500);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	sIntake->moveVoltage(0);
	logger->info("Turned to the basket. Shooting discs\n");
	pros::delay(50);

	sFlywheel->waitUntilSettled();
	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(50);
	sIntake->moveVoltage(0);
	if (sFlywheel->waitUntilSettled(1000)) {
		logger->warn("Flywheel timed out\n");
	} else {
		logger->info("Flywheel stabilized\n");
	}
	// pros::delay(600);

	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(50);
	sIntake->moveVoltage(0);
	if (sFlywheel->waitUntilSettled(1250)) {
		logger->warn("Flywheel timed out\n");
	} else {
		logger->info("Flywheel stabilized\n");
	}

	sFlywheel->shoot();

	logger->info("DONE. Time took: {}\n", (pros::micros() - startTime) / 1000.0 / 1000.0);
}
void test_auton() {
	sOdom->reset();

	sDrive->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	sDrive->setCurrentMotion(
	        std::make_unique<PIDTurn>(277.74, PID(4.6, 0.004, 3.5, true, 5), false, false, 0.5, 1.55));// tuning
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(0, PID(3, 0.0002, 2, true, 5), false, false, 0.5, 1.7));//
	// tuning sDrive->waitUntilSettled(); tuning profiled motions
}
void right_auton() {
	double D_VAR = 6;
	// -----------------------------------------------------------
	// right auton
	LoggerPtr logger = sLogger->createSource("Right Auto");
	uint32_t startTime = pros::micros();
	sOdom->reset();
	sDrive->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

	// sFlywheel->setVelocity(2488);// original
	sFlywheel->setVelocity(2450);// last minute fuckery
	sIntake->moveVoltage(12000);
	logger->info("Moving to intake first disc\n");
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(22.91, 50, 60, 60, 0.1));
	if (sDrive->waitUntilSettled(2500)) {
		logger->info("Done with movement to disc\n");
	} else {
		logger->warn("Movement to disc timed out\n");
	}

	// Pose goal = Pose(42, 104.7, 0);
	Pose goal = Pose(43.5, 103.2, 0);
	double turn_amount = sOdom->getCurrentState().position.headingToPoint(goal) * 180.0 / M_PI;

	pros::lcd::print(0, "turn amount %.2f", turn_amount);
	logger->info("Turn Amount To Basket: {}\n", turn_amount);
	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turn_amount, PID(160, 100, 0, true, 3)));// tuning

	// sDrive->setCurrentMotion(
	//         std::make_unique<PIDTurn>(turn_amount, PID(6.3, 0.004, 5.2, true, 5), false, false, 0.5, 1.92));// tuning

	sDrive->setCurrentMotion(
	        std::make_unique<PIDTurn>(turn_amount, PID(6.3, 0.004, 5.2, true, 5), false, false, 0.5, 1.95));// tuning

	if (sDrive->waitUntilSettled(1500)) {
		logger->info("Turn to basket done\n");
	} else {
		logger->warn("Turn to basket timed out\n");
	}
	sIntake->moveVoltage(0);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());

	sFlywheel->waitUntilSettled();
	// sFlywheel->triple_shoot(12000, -10000); // for 2 discs
	// sFlywheel->triple_shoot(12000, -12000, 750);

	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(50);
	sIntake->moveVoltage(0);
	// sFlywheel->waitUntilSettled(700);
	if (sFlywheel->waitUntilSettled(700)) {
		logger->warn("Flywheel timed out\n");
	} else {
		logger->warn("Flywheel stabilized\n");
	}
	// pros::delay(600);

	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(50);
	sIntake->moveVoltage(0);
	if (sFlywheel->waitUntilSettled(700)) {
		logger->warn("Flywheel timed out\n");
	} else {
		logger->warn("Flywheel stabilized\n");
	}
	// pros::delay(600);

	sFlywheel->shoot();
	logger->info("Done shooting discs\n");
	pros::delay(50);
	// sFlywheel->triple_shoot();

	// sFlywheel->setVelocity(2371.5);// for 2nd basket shot
	// sFlywheel->setVelocity(2400);// last minute fuckery
	sFlywheel->setVelocity(2420);// last minute fuckery

	// sFlywheel->shoot();
	// sIntake->moveVoltage(4000);
	// pros::delay(50);
	// sIntake->moveVoltage(0);
	// pros::delay(100);

	// // sFlywheel->shoot();
	// sFlywheel->triple_shoot();
	// sIntake->moveVoltage(4000);
	// pros::delay(50);
	// sIntake->moveVoltage(0);
	// pros::delay(200);
	// sFlywheel->waitUntilSettled();
	// sFlywheel->shoot();
	// sFlywheel->waitUntilSettled();
	// sFlywheel->shoot();
	// delay(200);

	// Pose endTarget = Pose(-20.5, 44.31);
	Pose endTarget = Pose(-22.2, 46.074);

	// sDrive->setCurrentMotion(
	//         std::make_unique<PIDTurn>(sOdom->getCurrentState().position.headingToPoint(endTarget) / M_PI * 180,
	//                                   PID(225, 100, 0, true, 2)));// tune this maybe - keep watch
	logger->info("Turn to line of 3\n");

	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(
	//         sOdom->getCurrentState().position.headingToPoint(endTarget) / M_PI * 180, PID(6.6, 0.002, D_VAR, true,
	//         5), false, false, 0.5, 1.84));// tune this maybe - keep watch

	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(
	        sOdom->getCurrentState().position.headingToPoint(endTarget) / M_PI * 180, PID(6.6, 0.002, D_VAR, true, 5),
	        false, false, 0.5, 1.9));// tune this maybe - keep watch

	//	sDrive->waitUntilSettled(2000);
	sDrive->waitUntilSettled(1500);
	sIntake->moveVoltage(12000);
	sDrive->setCurrentMotion(
	        std::make_unique<ProfiledMotion>(sOdom->getCurrentState().position.distanceTo(endTarget), 50, 60, 60, 0.1));
	sDrive->waitUntilSettled(2500);

	turn_amount = sOdom->getCurrentState().position.headingToPoint(goal) * 180.0 / M_PI;
	logger->info("Turn Amount To Basket: {}\n", turn_amount);
	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turn_amount, PID(85, 100, 0, true, 2)));// tuning
	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turn_amount, PID(6.3, 0.004, D_VAR, true, 7), false, false,
	// 0.5,
	//                                                    1.62));// tuning

	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turn_amount, PID(6.3, 0.004, D_VAR, true, 7), false, false, 0.5,
	                                                   1.7));// tuning

	sDrive->waitUntilSettled(2000);
	// sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("Done movement\n");
	sIntake->moveVoltage(0);
	pros::delay(100);

	sFlywheel->waitUntilSettled();
	// maybe change this to shoot individually - and add delay before first and second one
	// sFlywheel->triple_shoot(12000, -12000, 300);

	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(70);
	sIntake->moveVoltage(0);
	if (sFlywheel->waitUntilSettled(700)) {
		logger->warn("Flywheel timed out\n");
	} else {
		logger->warn("Flywheel stabilized\n");
	}
	// sFlywheel->waitUntilSettled();
	// pros::delay(600);

	sFlywheel->shoot();
	logger->info("Done with shooting\n");
	pros::delay(50);

	// -38.7 heading
	// x: 15.5 Y: -3.1
	// brake left side of drive - TURN HEADING TO -8.8
	// THEN JUST APPLY backwards drive pressure and do intake
	logger->info("Moving back to roller\n");
	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(-39, PID(88, 120, 0, true, 2)));
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(-39, PID(4.8, 0.0002, D_VAR, true, 5), false, false, 0.5, 1.85));

	//	sDrive->waitUntilSettled(2000);
	sDrive->waitUntilSettled();
	logger->info("Turn done\n");
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());

	Pose rollerPose = Pose(17.5, -5.1);
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(
	        -1 * sOdom->getCurrentState().position.distanceTo(rollerPose), 50, 60, 60, 1.5));
	sDrive->waitUntilSettled(2500);

	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(-8.8, PID(175, 100, 0, true, 2), true, false));
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(-8.8, PID(800, 100, 0, true, 2), true,
	                                                   false));// can prob increase P a lot because it doesnt matter
	sDrive->waitUntilSettled(250);
	sDrive->setCurrentMotion(std::make_unique<TimedMotion>(500, -7000));
	pros::delay(50);
	sIntake->moveVoltage(10000);
	pros::delay(300);
	sIntake->moveVoltage(0);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("DONE. Time took: {}\n", (pros::micros() - startTime) / 1000.0 / 1000.0);
}

void carry_auton() {
	LoggerPtr logger = sLogger->createSource("Carry Auto");
	uint32_t startTime = pros::micros();
	sOdom->reset();
	sDrive->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

	sFlywheel->setVelocity(2440);

	Pose goal = Pose(-15.801, 116.184);


	// move back and hit roller
	logger->info("Moving back to hit roller\n");
	sDrive->setCurrentMotion(std::make_unique<TimedMotion>(1500, -6000));
	pros::delay(100);
	logger->info("Rolling roller\n");
	sIntake->moveVoltage(10000);
	pros::delay(350);
	sIntake->moveVoltage(0);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("Done rolling roller\n");

	// Rotate to + intake 3 stack
	// Pose threeStackPose = Pose(32.6, 34.651);
	Pose threeStackPose = Pose(32.6, 37.651);
	double turnAmount = sOdom->getCurrentState().position.headingToPoint(threeStackPose) / M_PI * 180;
	logger->info("Arc Motion To Disk. Heading to: {}\n", turnAmount);
	sDrive->setCurrentMotion(
	        std::make_unique<PIDTurn>(turnAmount, PID(6.3, 0.004, 5.2, true, 5), false, true, 0.5, 2.2));// tuning
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("Arc Motion Done\n");

	logger->info("Move to intake 3 stack\n");
	sIntake->moveVoltage(12000);
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(
	        sOdom->getCurrentState().position.distanceTo(threeStackPose), 30, 60, 50, 0.25));
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("Done intaking 3 stack.\n");

	turnAmount = sOdom->getCurrentState().position.headingToPoint(goal) / M_PI * 180;
	logger->info("Rotate to basket: {}\n", turnAmount);
	sDrive->setCurrentMotion(
	        std::make_unique<PIDTurn>(turnAmount, PID(5.8, 0.02, 4.3, true, 5), false, false, 0.5, 1.9));// tuning
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("Done rotating to basket.\n");
	sIntake->moveVoltage(0);
	pros::delay(50);

	// shoot
	logger->info("Shooting discs\n");
	sFlywheel->waitUntilSettled();
	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(50);
	sIntake->moveVoltage(0);
	if (sFlywheel->waitUntilSettled(1250)) {
		logger->warn("Flywheel timed out\n");
	} else {
		logger->info("Flywheel stabilized\n");
	}

	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(50);
	sIntake->moveVoltage(0);
	if (sFlywheel->waitUntilSettled(1250)) {
		logger->warn("Flywheel timed out\n");
	} else {
		logger->info("Flywheel stabilized\n");
	}

	sFlywheel->shoot();
	logger->info("Done shooting discs\n");
	pros::delay(50);

	Pose endpoint = Pose(85.9, 97.4);
	//		 Pose endpoint = Pose(88.3, 95.4);
	logger->info("Turning to line of 3 + moving to roller\n");
	turnAmount = sOdom->getCurrentState().position.headingToPoint(endpoint) / M_PI * 180;
	sDrive->setCurrentMotion(
	        std::make_unique<PIDTurn>(turnAmount, PID(6.03, 0.002, 6.2, true, 5), false, false, 0.5, 1.5));// tuning
	sDrive->waitUntilSettled();
	// 47.5, 84.19, 84.17
	sIntake->moveVoltage(12000);
	sDrive->setCurrentMotion(
	        std::make_unique<ProfiledMotion>(sOdom->getCurrentState().position.distanceTo(endpoint), 50, 60, 60, 0.25));
	sDrive->waitUntilSettled();

	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(270, PID(6.6, 0.002, 6, true, 5), false, false, 0.5,
	                                                   1.45));// tune this maybe - keep watch
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<TimedMotion>(1000, -7000));
	pros::delay(250);
	sIntake->moveVoltage(10000);
	pros::delay(250);
	sIntake->moveVoltage(0);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("DONE. Time took: {}\n", (pros::micros() - startTime) / 1000.0 / 1000.0);
}

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
	switch (sRobot->getAuton()) {
		case Auton::LEFT:
			left_auton();
			break;
		case Auton::RIGHT:
			right_auton();
			break;
		case Auton::CARRY:
			carry_auton();
			break;
	}
}

void opcontrol() {
	// test_auton();
	sDrive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	sDrive->setCurrentMotion(std::make_unique<OpControlMotion>());
}
