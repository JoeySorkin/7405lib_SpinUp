#include "lib/physics/ProfiledTurn.h"
#include "Constants.h"
#include <cmath>


// !!!!!!!!!
// Code has issue - profile takes in dist, but you past in target
// - we have to calculate dist - but that requires knowing the pose of the robot at the start
// to get the shortest angle we have to turn
// but that also means we can only construct profile at the first time the motion is called
// !!!!!!!!!
ProfiledTurn::ProfiledTurn(double targetHeading, double omega, double alphaUp, double alphaDown, double threshold)
    : _targetHeading(targetHeading), _omega(omega), _alphaUp(alphaUp), _alphaDown(alphaDown), threshold(threshold),
      _profile(targetHeading, alphaUp, alphaDown, omega) {}

Motion::MotorVoltages ProfiledTurn::calculateVoltages(kinState state) {
	// set start time
	double turn;

	// get profile
	auto targ_state = _profile.getState((pros::millis() - startTime) / 1000.0);

	// !!!!!!!!!
	// Change from normalized values to using mV
	// !!!!!!!!!
	// calculate feedforward
	double FF;
	double kV = (1 / chassis::maxOmega);
	double kA = 1, kDe = 1;// tunable constants

	FF = (kV * targ_state.vel);
	FF += (targ_state.acc > 0) ? (kA * targ_state.acc) : (kDe * targ_state.acc);

	// calculate position PID
	double posPID;
	double pkP, pkI, pkD;
	double prev_pos_error;
	double pos_integral;

	double pos_error = state.position.getTheta() - targ_state.pos;
	pos_integral += pos_error;
	posPID = (pos_error * pkP) + (pos_error - prev_pos_error) * pkD + (pos_integral * pkI);
	prev_pos_error = pos_error;

	// calculate velocity PID
	double veloPID;
	double vkP, vkI, vkD;
	double prev_velo_error;
	double velo_integral;

	double velo_error = state.velocity().theta - targ_state.vel;
	velo_integral += velo_error;
	veloPID = (pos_error * pkP) + (pos_error - prev_pos_error) * pkD + (pos_integral * pkI);
	prev_velo_error = velo_error;

	// calculate turn
	turn = FF + posPID + veloPID;

	return {-turn, turn};
}

bool ProfiledTurn::isSettled(kinState state) {
	double abs_error = std::abs(_targetHeading - state.position.getTheta());
	return abs_error < threshold;
}