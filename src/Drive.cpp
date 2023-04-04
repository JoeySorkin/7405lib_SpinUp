#include "Drive.h"
#include "lib/physics/NullMotion.h"

Drive* Drive::INSTANCE = nullptr;

void Drive::initialize() {
	resetPosition();
	task = pros::c::task_create([](void* _) { sDrive->runner(_); }, nullptr, TASK_PRIORITY_DEFAULT,
	                            TASK_STACK_DEPTH_DEFAULT, "Drive");
}

void Drive::runner(void* ignored) {
	while (true) {
		// get current motion (make sure we destruct the motion)
		//        Motion* currentMotion = getCurrentMotion(200); //
		// TODO: add thread overload error thingy
		currentMotionMutex.take(TIMEOUT_MAX);
		// get computed motor voltages
		auto motorVolts = currentMotion->calculateVoltages(sOdom->getCurrentState());

		// set those
		setVoltageLeft(motorVolts.left);
		setVoltageRight(motorVolts.right);

		// check if motion has timed out (if it has, set current motion to nullmotion)
		if (isTimedOut) {
			logger->info("TIMED OUT\n");
			// printf("TIMED OUT \n");
			currentMotion = std::make_unique<NullMotion>();
			isTimedOut = false;
		}

		// isSettled = TODO: IMPLMEMENT THIS
		currentMotionMutex.give();
		pros::delay(20);
	}
}

bool Drive::waitUntilSettled(uint32_t timeout) {
	auto timer = Timeout(timeout);

	while (!timer.timedOut()) {
		if (isSettled.load()) {
			isSettled = false;
			return true;
		}
		pros::delay(20);
	}

	// Timed out
	isTimedOut = true;

	// wait until the Drive thread realizes we timed out
	while (isTimedOut) { pros::delay(20); }

	return false;
}

void Drive::setCurrentMotion(std::unique_ptr<Motion> motion) {
	currentMotionMutex.take(TIMEOUT_MAX);
	// destruct old motion to stop memory leaks
	currentMotion = std::move(motion);
	currentMotionMutex.give();
}

void Drive::setVoltageRight(int16_t voltage) {
	backRight.move(voltage);
	frontRight.move(voltage);
	middleRight.move(voltage);
}

void Drive::setVoltageLeft(int16_t voltage) {
	backLeft.move(voltage);
	frontLeft.move(voltage);
	middleLeft.move(voltage);
}

double Drive::getLeftPosition() {
	// return (middleLeft.get_position() + frontLeft.get_position() + backLeft.get_position()) / 3.0;
	return middleLeft.get_position();
}

double Drive::getRightPosition() {
	// return (middleRight.get_position() + frontRight.get_position() + backRight. get_position()) / 3.0;
	return middleRight.get_position();
}

void Drive::resetPosition() {
	backLeft.set_zero_position(0);
	backRight.set_zero_position(0);
	middleLeft.set_zero_position(0);
	middleRight.set_zero_position(0);
	frontLeft.set_zero_position(0);
	frontRight.set_zero_position(0);
}