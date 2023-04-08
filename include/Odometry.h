#pragma once

#include "lib/geometry/kinState.h"
#include "main.h"

#define sOdom Odometry::getInstance()

class Odometry {
private:
	pros::Rotation leftWheel, rightWheel, backWheel;
	pros::task_t odom_task;
	pros::Mutex stateMutex;

	double prev_l, prev_r, prev_b;

	kinState curr_state;

	static Odometry* INSTANCE;

	Odometry();
	Odometry(const Odometry&) = delete;
	Odometry& operator=(const Odometry&) = delete;

	void updatePosition(void* params);
	void printOdom();

public:
	static Odometry* getInstance() {
		if (!INSTANCE) { INSTANCE = new Odometry(); }
		return INSTANCE;
	}

	kinState getCurrentState();
	void initialize();
};