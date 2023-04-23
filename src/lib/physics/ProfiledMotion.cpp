#include "lib/physics/ProfiledMotion.h"
#include "Constants.h"
#include "Logger.h"
#include "Odometry.h"
#include "lib/controllers/PID.h"
#include "lib/utils/Math.h"
#include <cstdio>

LoggerPtr ProfiledMotion::logger = sLogger->createSource("ProfiledMotion", 0);

ProfiledMotion::ProfiledMotion(double dist, double maxVel, double accel, double decel, double threshold)
    : threshold(threshold), startPose(), profile(dist, accel, decel, maxVel) {
	// toggles whether or not debug info from this motion is logged
	// logger->setLevel(static_cast<LogSource::LogLevel>(LogSource::INFO | LogSource::WARNING | LogSource::ERROR));
}

void ProfiledMotion::start() {
	if (startTime == 0) {
		logger->debug("Motion start\n");
		startPose = sOdom->getCurrentState().position;
		Motion::start();
	}
}

Motion::MotorVoltages ProfiledMotion::calculateVoltages(kinState state) {
	double time = (pros::millis() - startTime) / 1000.0;
	auto targetState = profile.getState(time);

	// random values - and the divsor is inches/S or inches/S^2
	constexpr double kV = 12000.0 / chassis::maxVel;
	constexpr double kA = 12000.0 / 250.0; //random # for now
	constexpr double kDe = 12000.0 / 650.0;
	PID posPID = PID(600.0, 0, 0);

	double distTraveled = sOdom->getCurrentState().position.distanceTo(startPose) * util::sign(targetState.pos);
	double error = -1 * (distTraveled - targetState.pos);
	double fbPwr = posPID(error);

	double FF = kV * targetState.vel;
	if(distTraveled > 0){
		FF += targetState.acc > 0 ? kA * targetState.acc : kDe * targetState.acc;
	}
	else {
		FF += targetState.acc > 0 ? kDe * targetState.acc : kA * targetState.acc;
	}
	double power = FF + fbPwr;

	logger->debug(
	        "TIME: {} State: pos: {} vel: {:.2f} acc: {:.2f} err: {:.2f} dist traveled: {:.2f} ff: {:.2f} fb: {:.2f} "
	        "total pwr: {:.2f} "
	        "\n",
	        time, targetState.pos, targetState.vel, targetState.acc, error, distTraveled, FF, fbPwr, power);

	return {power, power};
}

bool ProfiledMotion::isSettled(kinState state) {
	double distTraveled = sOdom->getCurrentState().position.distanceTo(startPose);
	return std::abs(profile.getTargetDist() - distTraveled) < threshold;
}