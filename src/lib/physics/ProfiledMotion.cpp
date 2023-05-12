#include "lib/physics/ProfiledMotion.h"
#include "Constants.h"
#include "Logger.h"
#include "Odometry.h"
#include "lib/utils/Math.h"

LoggerPtr ProfiledMotion::logger = sLogger.createSource("ProfiledMotion", 0);

ProfiledMotion::ProfiledMotion(double dist, double maxVel, double accel, double decel, double threshold)
    : threshold(threshold), errorSum(0), startPose(), profile(dist, accel, decel, maxVel), prevSign(dist >= 0) {
	// toggles whether or not debug info from this motion is logged
	// logger->setLevel(static_cast<LogSource::LogLevel>(LogSource::INFO | LogSource::WARNING | LogSource::ERROR));
}

void ProfiledMotion::start() {
	if (startTime == 0) {
		logger->debug("Motion start. Total Time: {}\n", profile.getTotalTime());
		startPose = sOdom.getCurrentState().position;
		Motion::start();
	}
}

Motion::MotorVoltages ProfiledMotion::calculateVoltages(kinState state) {
	double time = (pros::millis() - startTime) / 1000.0;
	auto targetState = profile.getState(time);

	// random values - and the divsor is inches/S or inches/S^2
	// constexpr double kV = 12000 / chassis::maxVel;
	// constexpr double kV = 157.135;
	// constexpr double kV = 12000 / 67.0;
	// constexpr double kA = 70.7333777722;
	// constexpr double kA = 52.5;
	// constexpr double kDe = 42.73;
	// constexpr double kDe = 49.73;

	// constexpr double kVLeft = 146.4539999079319;// original

	// constexpr double kALeft = 35.450363464220203;// worked at 20 inches well
	// constexpr double kALeft =
	//         37.450363464220203;// works well for all distances when max vel is 50, max accel and max decel are
	//         60
	// constexpr double kALeft = 36.75;// works well for all distances when max vel is 50, max
	// accel and max decel are 60
	// constexpr double kDeLeft = 36.55;


	// testing
	constexpr double kSLeft = 1065.65839038;    // original
	constexpr double kVLeft = 152.4539999079319;// original
	// constexpr double kVLeft = 157.4539999079319;
	constexpr double kALeft = 35.450363464220203;
	constexpr double kDeLeft = 38.5;
	// constexpr double kDeLeft = kALeft;

	constexpr double kSRight = 835.517;
	// constexpr double kVRight = 157.135;
	constexpr double kVRight = 153.535;
	constexpr double kARight = 35.3875;
	// constexpr double kDeRight = 35;
	constexpr double kDeRight = kARight;

	// original
	// constexpr double kSLeft = 1065.65839038;// original
	// constexpr double kVLeft = 146.4539999079319;
	// constexpr double kALeft = 24.450363464220203;
	// constexpr double kDeLeft = kALeft;

	// constexpr double kSRight = 835.517;
	// constexpr double kVRight = 157.135;
	// constexpr double kARight = 35.3875;
	// constexpr double kDeRight = kARight;

	int sign = util::sign(targetState.vel);
	if (util::fpEquality(targetState.vel, 0.0)) { sign = 0; }

	double FFLeft = kVLeft * targetState.vel + sign * kSLeft;
	FFLeft += targetState.acc > 0 ? kALeft * targetState.acc : kDeLeft * targetState.acc;

	double FFRight = kVRight * targetState.vel + sign * kSRight;
	FFRight += targetState.acc > 0 ? kARight * targetState.acc : kDeRight * targetState.acc;

	auto curState = sOdom.getCurrentState();
	double distTraveled = curState.position.distanceTo(startPose) * util::sign(targetState.pos);
	double error = -1 * (distTraveled - targetState.pos);

	// TODO: For wednesday - take out FB pwr first and just tune the FF values first - minimize drift and get as close
	// as possible
	// double fbPwr = 0;
	double fbPwr = error * 750;
	// double fbPwr = error > 0 ? error * 500 : error * 250;
	if (std::abs(profile.getTargetDist() - distTraveled) < 2) { errorSum += error; }
	if (util::sign(error) != prevSign) {
		errorSum = 0;
		prevSign = util::sign(error);
	}

	fbPwr += errorSum * 250;

	double leftPwr = FFLeft + fbPwr;
	double rightPwr = FFRight + fbPwr;

	logger->debug("TIME: {} tpos: {:.2f} tvel: {:.2f} cvel: {:.2f} tacc: {:.2f} err: {:.2f} dist traveled: {:.2f} "
	              "ff: {:.2f} fb: {:.2f} "
	              "total pwr: {:.2f} curHeading: {} x: {} err sum: {}"
	              "\n",
	              time, targetState.pos, targetState.vel, curState.velocity().y, targetState.acc, error, distTraveled,
	              FFLeft, fbPwr, leftPwr, curState.position.getTheta() / M_PI * 180, curState.position.getX(),
	              errorSum);

	return {leftPwr, rightPwr};
}

bool ProfiledMotion::isSettled(kinState state) {
	double distTraveled = sOdom.getCurrentState().position.distanceTo(startPose);
	return std::abs(profile.getTargetDist() - distTraveled) < threshold;
}