#include "main.h"
#include "Drive.h"
#include "Logger.h"
#include "Odometry.h"
#include "Robot.h"
#include "Display.h"
#include "lib/controllers/PID.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/kinState.h"
#include "lib/physics/NullMotion.h"
#include "lib/physics/OpControlMotion.h"
#include "lib/physics/PIDMotion.h"
#include "lib/physics/PIDTurn.h"
#include "lib/physics/ProfiledMotion.h"
#include "lib/physics/TimedMotion.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <ctime>
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
	// Display::guiInitialize();
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

void rightAuton(){
	sRobot->setOpMode(Robot::AUTONOMOUS);
	sIntake->moveVoltage(12000);
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(20.0));
	sDrive->waitUntilSettled();

	Pose goal = Pose(40.0, 93.146); //1.77
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(
		(180/M_PI) * (sOdom->getCurrentState().position.headingToPoint(goal)), 
		PID(500, 50.0, 800, true, 5)));
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(-7.0));
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(10.0));
	pros::delay(250);
	sShooter->fireScata();
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(-3.0));
	sDrive->waitUntilSettled();

	Pose threeStack = Pose(-24.6176, 42.9041); //0.6322	
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(
		(180.0/M_PI) * (sOdom->getCurrentState().position.headingToPoint(threeStack)), 
		PID(300.0, 50.0, 1200.0, true, 5.0)));
	pros::delay(1000);
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(sOdom->getCurrentState().position.distanceTo(threeStack)));
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(-10.0));
	sDrive->waitUntilSettled();

	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(
		(180.0/M_PI) * (sOdom->getCurrentState().position.headingToPoint(goal)), 
		PID(300.0, 100.0, 1200.0, true, 5)));
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(6.0));
	pros::delay(120);
	sShooter->fireScata();
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(-4.0));
	sDrive->waitUntilSettled();
	sIntake->moveVoltage(0);

	Pose roller = Pose(16.0, 2.0);
	printf("roller target heading: %.2f\n", 180 - ((180/M_PI) * (sOdom->getCurrentState().position.headingToPoint(roller))));
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(
		 180 + (180/M_PI) * (sOdom->getCurrentState().position.headingToPoint(roller)),
		PID(800.0, 50.0, 1200.0, true, 5.0)));
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(5.0));
	sDrive->setCurrentMotion(std::make_unique<ProfiledMotion>(-sOdom->getCurrentState().position.distanceTo(roller)));
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<PIDTurn>(
		0.0,
		PID(800.0, 50.0, 1200.0, true, 5.0)));
	sDrive->waitUntilSettled();
	sDrive->setCurrentMotion(std::make_unique<TimedMotion>(300, -6000));
	sDrive->waitUntilSettled();
	sIntake->moveVoltage(12000);
	pros::delay(100);
	sIntake->moveVoltage(0);
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
	LoggerPtr logger = sLogger->createSource("AUTONOMOUS");
	sRobot->setOpMode(Robot::AUTONOMOUS);
// 	switch (Display::getAutonMode()) {
//     case Display::ALPHA:
//       // nothing
//       break;
//     case Display::BETA:
//       // Right side / Red
//     //   leftAuton();
//       break;
//     case Display::GAMMA:
//       // left side / blue
//       rightAuton();
//       break;
//     case Display::DELTA:
//       //add another auton if you want
//     //   carryAuton();
//       break;
//     case Display::OMEGA:
//       // eeeee
//     //   skillsAuton();
//       break;
//   }

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
	sRobot->setOpMode(Robot::AUTONOMOUS);
	rightAuton();
}