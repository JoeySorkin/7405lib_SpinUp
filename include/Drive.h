#pragma once

#include "Logger.h"
#include "Odometry.h"
#include "lib/physics/Motion.h"
#include "lib/utils/Timeout.h"
#include "main.h"
#include "pros/motors.h"
#include <atomic>
#include <memory>

#define sDrive Drive::getInstance()
class Drive {
private:
	Drive() = default;
	Drive(const Drive&) = delete;
	Drive& operator=(const Drive&) = delete;

	void runner(void* ignored);

	// Motors
	pros::Motor frontLeft{ports::frontLeftMotor, true}, frontRight{ports::frontRightMotor, false},
	        middleLeft{ports::middleLeftMotor, false}, middleRight{ports::middleRightMotor, true},
	        backLeft{ports::backLeftMotor, true}, backRight{ports::backRightMotor, false};

	// Motor Control
	void setVoltageLeft(int16_t voltage);
	void setVoltageRight(int16_t voltage);

	// General Stuff
	static Drive* INSTANCE;
	pros::task_t task;
	LoggerPtr logger;

	// thread safety
	std::atomic<bool> isSettled = false;
	std::atomic<bool> isTimedOut = false;
	pros::Mutex currentMotionMutex;
	std::unique_ptr<Motion> currentMotion;


public:
	static Drive* getInstance() {
		if (!INSTANCE) { INSTANCE = new Drive(); }

		return INSTANCE;
	}
	void initialize();
	void setCurrentMotion(std::unique_ptr<Motion> motion);

	double getLeftPosition();
	double getRightPosition();
	void resetPosition();

	void setBrakeMode(pros::motor_brake_mode_e_t mode);
	void setBrakeModeLeft(pros::motor_brake_mode_e_t mode);
	void setBrakeModeRight(pros::motor_brake_mode_e_t mode);
	pros::motor_brake_mode_e_t getBrakeMode() const;

	// Control
	/**
	 * @return returns true if settled, false if timed out
	 */
	bool waitUntilSettled(uint32_t timeout = TIMEOUT_MAX);
};