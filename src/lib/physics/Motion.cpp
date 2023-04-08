//
// Created by Joey Sorkin on 3/24/23.
//

#include "lib/physics/Motion.h"

void Motion::start() {
	if (startTime == 0) { startTime = pros::millis(); }
};
