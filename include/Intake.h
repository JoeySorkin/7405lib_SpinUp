#pragma once
#include "main.h"
#include "pros/motors.hpp"

#define sIntake Intake::getInstance()

class Intake {
private:
	pros::Motor_Group motors;

	Intake();
	Intake(const Intake&) = delete;
	Intake& operator=(const Intake&) = delete;

public:
	inline static Intake& getInstance() {
		static Intake INSTANCE;

		return INSTANCE;
	}

	void initialize();

	void moveVoltage(int mv);
	void moveVel(int vel);
	void brake();
};