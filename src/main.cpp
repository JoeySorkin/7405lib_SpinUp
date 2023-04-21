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

void opcontrol() {

	sDrive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	sDrive->setCurrentMotion(std::make_unique<OpControlMotion>());
	LoggerPtr logger = sLogger->createSource("OpControl");

	// left auton
	uint32_t startTime = pros::micros();
	sFlywheel->setVelocity(2610);
	sOdom->reset();

	Pose goal = Pose(-10.3, 115.32, 0);
	// Pose goal = Pose(-5.3, 115.32, 0);

	// move back and hit roller
	logger->info("Moving back to hit roller\n");
	sDrive->setCurrentMotion(std::make_unique<TimedMotion>(1500, -6000));
	pros::delay(100);
	logger->info("Rolling roller\n");
	sIntake->moveVoltage(10000);
	pros::delay(300);
	sIntake->moveVoltage(0);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("Done rolling roller\n");

	logger->info("Arc Motion To Disk + Move to intake disc\n");
	// -45.7
	// trying -43 first
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(-43, PID(185, 70, 0, true, 2), true, false, 0.5));
	sDrive->waitUntilSettled();
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
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turnAmount, PID(140, 60, 20, true, 2), false, false, 0.5));
	sDrive->waitUntilSettled(1500);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("Turned to basket. Shooting discs\n");

	sFlywheel->waitUntilSettled();
	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(50);
	sIntake->moveVoltage(0);
	// sFlywheel->waitUntilSettled(700);
	if (sFlywheel->waitUntilSettled(700)) {
		logger->warn("Flywheel timed out\n");
	} else {
		logger->info("Flywheel stabilized\n");
	}
	// pros::delay(600);

	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(50);
	sIntake->moveVoltage(0);
	if (sFlywheel->waitUntilSettled(700)) {
		logger->warn("Flywheel timed out\n");
	} else {
		logger->info("Flywheel stabilized\n");
	}

	sFlywheel->shoot();
	pros::delay(50);
	logger->info("Done shooting discs\n");

	logger->info("Rotating + Intaking 3 Stack\n");
	Pose threeStack = Pose(35.66, 33.728);
	turnAmount = sOdom->getCurrentState().position.headingToPoint(threeStack) / M_PI * 180;
	logger->info("Turn Amount to 3 stack: {}\n", turnAmount);
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turnAmount, PID(110, 85, 0, true, 2), false, false, 0.5));
	sDrive->waitUntilSettled(1500);
	sIntake->moveVoltage(12000);
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(threeStack.distanceTo(sOdom->getCurrentState().position),
	                                                          30, 60, 40, 0.25));
	sDrive->waitUntilSettled(2500);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	sIntake->moveVoltage(0);

	turnAmount = sOdom->getCurrentState().position.headingToPoint(goal) / M_PI * 180;
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turnAmount, PID(95, 85, 0, true, 2), false, false, 0.5));
	sDrive->waitUntilSettled(2500);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());

	logger->info("Turned to the basket. Shooting discs\n");

	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(50);
	sIntake->moveVoltage(0);
	if (sFlywheel->waitUntilSettled(700)) {
		logger->warn("Flywheel timed out\n");
	} else {
		logger->info("Flywheel stabilized\n");
	}
	// pros::delay(600);

	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(50);
	sIntake->moveVoltage(0);
	if (sFlywheel->waitUntilSettled(700)) {
		logger->warn("Flywheel timed out\n");
	} else {
		logger->info("Flywheel stabilized\n");
	}

	sFlywheel->shoot();
	logger->info("Done shooting discs\n");

	logger->info("DONE. Time took: {}\n", (pros::micros() - startTime) / 1000.0 / 1000.0);

	// delete after we get done with autons
	pros::delay(100);
	sLogger->terminate();// just to get rid of logging info for time while making autons


	// right auton
	/*uint32_t startTime = pros::micros();
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


	// sFlywheel->setVelocity(2505);
	// sFlywheel->setVelocity(2560);
	sFlywheel->setVelocity(2495);// tuning
	sIntake->moveVoltage(12000);
	logger->info("Moving to intake first disc\n");
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(22.91, 50, 60, 60, 0.1));
	if (sDrive->waitUntilSettled(2500)) {
	    logger->info("Done with movement to disc\n");
	} else {
	    logger->warn("Movement to disc timed out\n");
	}

	Pose goal = Pose(41, 106, 0);
	double turn_amount = sOdom->getCurrentState().position.headingToPoint(goal) * 180.0 / M_PI;

	pros::lcd::print(0, "turn amount %.2f", turn_amount);
	logger->info("Turn Amount To Basket: {}\n", turn_amount);
	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turn_amount, PID(160, 100, 0, true, 3)));// tuning
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turn_amount, PID(160, 100, 0, true, 3)));// tuning
	if (sDrive->waitUntilSettled(2000)) {
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

	// sFlywheel->setVelocity(2330);// for 2nd basket shot
	sFlywheel->setVelocity(2370);// for 2nd basket shot

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
	sDrive->setCurrentMotion(
	        std::make_unique<PIDTurn>(sOdom->getCurrentState().position.headingToPoint(endTarget) / M_PI * 180,
	                                  PID(180, 50, 20, true, 5)));// tune this maybe - keep watch
	sDrive->waitUntilSettled(2000);
	sIntake->moveVoltage(12000);
	sDrive->setCurrentMotion(
	        std::make_unique<ProfiledMotion>(sOdom->getCurrentState().position.distanceTo(endTarget), 50, 60, 60, 0.1));
	sDrive->waitUntilSettled(2500);

	turn_amount = sOdom->getCurrentState().position.headingToPoint(goal) * 180.0 / M_PI;
	logger->info("Turn Amount To Basket: {}\n", turn_amount);
	// sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turn_amount, PID(85, 100, 0, true, 2)));// tuning
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(turn_amount, PID(80, 50, 0, true, 5)));// tuning
	sDrive->waitUntilSettled(2000);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("Done movement\n");
	sIntake->moveVoltage(0);
	pros::delay(100);

	sFlywheel->waitUntilSettled();
	// maybe change this to shoot individually - and add delay before first and second one
	// sFlywheel->triple_shoot(12000, -12000, 300);

	sFlywheel->shoot();
	sIntake->moveVoltage(4000);
	pros::delay(50);
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
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(-39, PID(88, 140, 0, true, 2)));
	sDrive->waitUntilSettled(2000);
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
	sDrive->setCurrentMotion(std::make_unique<TimedMotion>(500, -9000));
	pros::delay(50);
	sIntake->moveVoltage(10000);
	pros::delay(400);
	sIntake->moveVoltage(0);
	sDrive->setCurrentMotion(std::make_unique<NullMotion>());
	logger->info("DONE. Time took: {}\n", (pros::micros() - startTime) / 1000.0 / 1000.0);

	sDrive->setCurrentMotion(std::make_unique<NullMotion>());// tuning
	sDrive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	sFlywheel->setVelocity(0);*/
	//	{H: 29.85, 0.053221, 22.8943216}
	//	{H: 48.7249, 0.703349, 70.381138}
}
