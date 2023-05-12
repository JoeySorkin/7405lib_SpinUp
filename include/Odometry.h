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
	double curL, curR;
	double leftVel, rightVel;

	kinState curr_state;

	Odometry();
	Odometry(const Odometry&) = delete;
	Odometry& operator=(const Odometry&) = delete;

	void updatePosition(void* params);
	void printOdom(kinState state);

public:
	inline static Odometry& getInstance() {
		static Odometry INSTANCE;

		return INSTANCE;
	}

	kinState getCurrentState();
	double getLeftVel();
	double getLeftPos();
	double getRightVel();
	double getRightPos();
	void initialize();
	void reset();
};