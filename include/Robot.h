#pragma once
#include "main.h"
#include <atomic>
#define sRobot Robot::getInstance()

enum class Auton : uint32_t { NONE = 0, LEFT, RIGHT, CARRY };

class Robot {
public:
	enum OpMode { DRIVER, AUTONOMOUS };

private:
	// Singleton Stuff
	static Robot* INSTANCE;
	Robot() = default;
	Robot(const Robot&) = delete;
	Robot& operator=(const Robot&) = delete;

	// OpMode
	std::atomic<OpMode> opmode = AUTONOMOUS;
	std::atomic<Auton> auton = Auton::NONE;

public:
	void initialize();

	//  OpMode
	OpMode getOpMode();
	void setOpMode(OpMode op_mode);

	// Auton
	Auton getAuton();
	void setAuton(Auton auton);

	// Singleton Stuff
	static Robot* getInstance() {
		if (!INSTANCE) { INSTANCE = new Robot(); }
		return INSTANCE;
	}
};