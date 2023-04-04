//
// Created by Joey Sorkin on 3/24/23.
//

#include "lib/physics/Motion.h"

void Motion::start() {
	if (start_time == 0) start_time = pros::millis();
};
