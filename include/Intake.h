#pragma once
#include "main.h"
#include "pros/motors.hpp"

#define sIntake Intake::getInstance()

class Intake {
private:
	static Intake* INSTANCE;
	pros::Motor_Group motors;

	Intake();
	Intake(const Intake&) = delete;
	Intake& operator=(const Intake&) = delete;

public:
	static Intake* getInstance() {
		if (!INSTANCE) { INSTANCE = new Intake(); }

		return INSTANCE;
	}

	void initialize();

	void moveVoltage(int mv);
	void moveVel(int vel);
	void brake();
};