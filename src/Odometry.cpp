#include "Odometry.h"
#include "Constants.h"
#include "Drive.h"
#include "lib/geometry/Pose.h"
#include "lib/geometry/kinState.h"
#include "pros/rtos.h"
#include <cmath>
#include <cstdio>
#include <math.h>
#include <string>
#include <type_traits>

Odometry::Odometry()
    : prev_b(0), prev_l(0), prev_r(0), leftWheel(ports::leftRotation), rightWheel(ports::rightRotation),
      backWheel(ports::backRotation) {
	//	backWheel.reset();
	//	backWheel.set_data_rate(5);
	leftWheel.reset_position();
	leftWheel.set_data_rate(5);
	rightWheel.reset_position();
	rightWheel.set_data_rate(5);
#ifdef TEAM_K
	rightWheel.reverse();
#endif
}


void Odometry::initialize() {
	prev_l = 0;
	prev_b = 0;
	prev_r = 0;
	curr_state = kinState({0, 0, 0}, {0, 0, 0}, {0, 0, 0});
	odom_task = pros::c::task_create([](void* _) { sOdom.updatePosition(_); }, nullptr, TASK_PRIORITY_DEFAULT,
	                                 TASK_STACK_DEPTH_DEFAULT, "Odometry Task");
	leftWheel.reset();
	rightWheel.reset();
}

void Odometry::reset() {
	curr_state = kinState({0, 0, 0}, {0, 0, 0}, {0, 0, 0});
	leftWheel.reset();
	rightWheel.reset();
}
void Odometry::updatePosition(void* params) {

	while (true) {
		uint32_t time = pros::millis();

		double l_dist;
		double r_dist;
		double b_dist;
		double perp_offset;

		// if rotation sensor port is defined, then we use that to get dist traveled
		// otherwise we use motor encoders
		// Marked constexpr so it is a compile time thing
		if constexpr (ports::leftRotation > 0) {
			double LE = leftWheel.get_position();
			curL = LE / 36000.0 * M_PI * odometers::leftDeadwheelDiameter;
			l_dist = ((LE - prev_l) / 36000.0) * M_PI * odometers::leftDeadwheelDiameter;
			prev_l = LE;
		} else {
			double LE = sDrive.getLeftPosition();
			l_dist = ((LE - prev_l) / 360.0) * M_PI * odometers::leftDeadwheelDiameter;
			prev_l = LE;
		}

		if constexpr (ports::rightRotation > 0) {
			double RE = rightWheel.get_position();
			curR = RE / 36000.0 * M_PI * odometers::leftDeadwheelDiameter;
			r_dist = ((RE - prev_r) / 36000.0) * M_PI * odometers::rightDeadwheelDiameter;
			prev_r = RE;
		} else {
			double RE = sDrive.getRightPosition();
			r_dist = ((RE - prev_r) / 360.0) * M_PI * odometers::rightDeadwheelDiameter;
			prev_r = RE;
		}

		double dh = (l_dist - r_dist) / (odometers::trackWidth);

		if constexpr (ports::backRotation > 0) {
			double BE = backWheel.get_position();
			b_dist = ((BE - prev_b) / 36000.0) * M_PI * odometers::backDeadwheelDiameter;
			prev_b = BE;

			perp_offset = b_dist + (odometers::backOffset * dh);// need to test this
		} else {
			b_dist = 0;
			perp_offset = 0;
		}

		// idk why Kylan put this here but I'm keeping it here for now
		perp_offset = 0;

		double distance =
		        (dh == 0) ? (l_dist + r_dist) / 2.0 : ((l_dist + r_dist) / dh) * sin(dh / 2.0);// also need to test this

		double dx = distance * cos(curr_state.position.getTheta() - (M_PI / 2)) +
		            perp_offset * sin(curr_state.position.getTheta() + (M_PI / 2));
		double dy = distance * sin(curr_state.position.getTheta() + (M_PI / 2)) +
		            perp_offset * cos(curr_state.position.getTheta() + (M_PI / 2));
		double dt = 20.0 / 1000.0;

		stateMutex.take(TIMEOUT_MAX);
		curr_state.setAcceleration((curr_state.velocity().x - (dx / dt)) / dt,
		                           (curr_state.velocity().y - (dy / dt)) / dt,
		                           (curr_state.velocity().theta - (dh / dt)) / dt);
		curr_state.setVelocity(dx / dt, dy / dt, dh / dt);

		Pose prevPose = curr_state.position;

		curr_state.position = {curr_state.position.getX() + dx, curr_state.position.getY() + dy,
		                       curr_state.position.getTheta() + dh};

		leftVel = l_dist / dt;
		rightVel = r_dist / dt;

		kinState state = curr_state;
		stateMutex.give();
		printOdom(state);// this don't need to be in mutex - or it wastes time that we could've released mutex for other
		                 // threads
		pros::c::task_delay_until(&time, 20);
	}
}

double Odometry::getLeftVel() {
	return leftVel;
}

double Odometry::getLeftPos() {
	return curL;
}

double Odometry::getRightVel() {
	return rightVel;
}

double Odometry::getRightPos() {
	return curR;
}

void Odometry::printOdom(kinState state) {
	pros::lcd::set_text(1, "gH: " + std::to_string((180 / M_PI) * state.position.getTheta()));
	pros::lcd::set_text(2, "gX: " + std::to_string(state.position.getX()));
	pros::lcd::set_text(3, "gY: " + std::to_string(state.position.getY()));
}

kinState Odometry::getCurrentState() {

	stateMutex.take(TIMEOUT_MAX);
	kinState p = curr_state;
	stateMutex.give();
	return p;
}