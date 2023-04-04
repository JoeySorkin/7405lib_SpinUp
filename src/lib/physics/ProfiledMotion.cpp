#include "lib/physics/ProfiledMotion.h"
#include "Constants.h"
#include "lib/controllers/PID.h"
#include "pros/rtos.hpp"

ProfiledMotion::ProfiledMotion(double dist, double maxVel, double accel, double decel, double threshold)
    : profile(dist, accel, decel, maxVel) {}

Motion::MotorVoltages ProfiledMotion::calculateVoltages(kinState state) {
	auto targetState = profile.getState(pros::millis() - start_time);

	constexpr double kV = 1 / chassis::maxVel;
	constexpr double kA = 1;
	constexpr double kDe = 1;

	double FF = kV * targetState.vel;
	FF += targetState.acc > 0 ? kA * targetState.acc : kDe * targetState.acc;

	PID posPID = PID(0, 0, 0);
	double fbPwr = posPID(/*TODO: Determine how we want to calculate dist that robot traveled*/ 0.0);

	double power = FF + fbPwr;

	return {power, power};
}

bool ProfiledMotion::isSettled(kinState state) {
	// TODO: determine best way to see if we moved to the position - because without modifying odom, there's no nice way
	// of  knowing how far we've traveled besides maybe hypot from our start pose and current pose, but that's sketch af
}