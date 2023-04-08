
#include "main.h"
#include <atomic>

#define sRobot Robot::getInstance()
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

public:
	void initialize();

	//  OpMode
	OpMode getOpMode();
	void setOpMode(OpMode op_mode);

	// Singleton Stuff
	static Robot* getInstance() {
		if (!INSTANCE) { INSTANCE = new Robot(); }
		return INSTANCE;
	}
};