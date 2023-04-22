#include "Drive.h"
#include "Controller.h"
#include "lib/physics/MaxAccMotion.h"
#include "lib/physics/NullMotion.h"
#include "pros/rtos.hpp"
#include <cstdio>

Drive* Drive::INSTANCE = nullptr;

void Drive::initialize() {
	currentMotion = std::make_unique<NullMotion>();
	// currentMotion = std::make_unique<MaxAccMotion>();
	INSTANCE->logger = sLogger->createSource("Drive");
	resetPosition();
	task = pros::c::task_create([](void* _) { sDrive->runner(_); }, nullptr, TASK_PRIORITY_DEFAULT,
	                            TASK_STACK_DEPTH_DEFAULT, "Drive");
}

void Drive::runner(void* ignored) {
	// uint32_t startTime = pros::micros();
	while (true) {
		// get current motion (make sure we destruct the motion)
		//        Motion* currentMotion = getCurrentMotion(200); //
		// TODO: add thread overload error thingy
		currentMotionMutex.take(TIMEOUT_MAX);

		// TODO: change how motion.start() is handeled - idk how you guys want to do it so i just have a temp thing
		currentMotion->start();

		// check if motion has timed out (if it has, set current motion to nullmotion)
		if (isTimedOut.load()) {
			logger->info("TIMED OUT\n");

			currentMotion = std::make_unique<NullMotion>();
			isTimedOut.store(false);
		}

		// get computed motor voltages
		auto motorVolts = currentMotion->calculateVoltages(sOdom->getCurrentState());

		// set those
		setVoltageLeft(motorVolts.left);
		setVoltageRight(motorVolts.right);

		isSettled.store(currentMotion->isSettled(sOdom->getCurrentState()));

		// double driveVoltage = (backLeft.get_voltage() + middleLeft.get_voltage() + frontLeft.get_voltage() +
		//    backRight.get_voltage() + middleRight.get_voltage() + frontRight.get_voltage()) /
		//   6.0 / 1000.0;

		// auto state = sOdom->getCurrentState();
		// double timestamp = (pros::micros() - startTime) / 1000.0 / 1000.0;

		// timestamp (s), voltage (V), vel (in/S), acc (in/S^2)
		// printf("%f,%f,%f,%f\n", timestamp, driveVoltage, state.velocity().y, state.acceleration().y);

		currentMotionMutex.give();
		pros::delay(20);
	}
}

bool Drive::waitUntilSettled(uint32_t timeout) {
	auto timer = Timeout(timeout);

	while (!timer.timedOut()) {
		if (isSettled.load()) {
			isSettled.store(false);
			return true;
		}

		pros::delay(20);
	}

	// Timed out
	isTimedOut.store(true);

	// wait until the Drive thread realizes we timed out
	while (isTimedOut.load()) { pros::delay(20); }

	return false;
}

void Drive::setCurrentMotion(std::unique_ptr<Motion> motion) {
	currentMotionMutex.take(TIMEOUT_MAX);
	// destruct old motion to stop memory leaks
	currentMotion = std::move(motion);
	currentMotionMutex.give();
}

void Drive::setVoltageRight(int16_t voltage) {
	backRight.move_voltage(voltage);
	frontRight.move_voltage(voltage);
	middleRight.move_voltage(voltage);
}

void Drive::setVoltageLeft(int16_t voltage) {
	backLeft.move_voltage(voltage);
	frontLeft.move_voltage(voltage);
	middleLeft.move_voltage(voltage);
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