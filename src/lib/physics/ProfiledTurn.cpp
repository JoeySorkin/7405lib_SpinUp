#include "lib/physics/ProfiledTurn.h"
#include "Constants.h"
#include <cmath>


// !!!!!!!!!
// Code has issue - profile takes in dist, but you passed in target
// - we have to calculate dist - but that requires knowing the pose of the robot at the start
// to get the shortest angle we have to turn
// but that also means we can only construct profile at the first time the motion is called
// !!!!!!!!!
ProfiledTurn::ProfiledTurn(double targetHeading, double omega, double alphaUp, double alphaDown, double threshold)
    : _targetHeading((targetHeading * M_PI) / 180), _omega(omega), _alphaUp(alphaUp), _alphaDown(alphaDown), threshold(threshold),
      _profile((targetHeading * M_PI) / 180, alphaUp, alphaDown, omega) {}

Motion::MotorVoltages ProfiledTurn::calculateVoltages(kinState state) {
	// set start time
	double turn;

	// get profile
	auto targ_state = _profile.getState((pros::millis() - startTime) / 1000.0);

	// calculate feedforward
	double FF;
	double kV = (12000.0 / chassis::maxOmega);
	double kA = 550, kDe = 200;// tunable constants

	FF = (kV * targ_state.vel);
	printf("Time: %.2f, FF Power%.2f \t", ((pros::millis() - startTime) / 1000.0), FF);

	FF += (targ_state.acc > 0) ? (kA * targ_state.acc) : (kDe * targ_state.acc);

	// calculate position PID
	double posPID;
	double pkP = 1000, pkI = 0, pkD;
	double prev_pos_error;
	double pos_integral;

	double pos_error = state.position.getTheta() - targ_state.pos;
	pos_integral += pos_error;
	posPID = (pos_error * pkP) + (pos_error - prev_pos_error) * pkD + (pos_integral * pkI);
	prev_pos_error = pos_error;

	printf("Position T:A  %.2f, %.2f\t", targ_state.pos, state.position.getTheta());
	printf("Velocity T:A  %.2f, %.2f\t", targ_state.vel, state.velocity().theta);
	printf("Acceleration T:A  %.2f, %.2f\t", targ_state.acc, state.acceleration().theta);


	// calculate velocity PID
	double veloPID;
	double vkP = 200, vkI, vkD;
	double prev_velo_error;
	double velo_integral;

	double velo_error = state.velocity().theta - targ_state.vel;
	velo_integral += velo_error;
	veloPID = (velo_error * vkP) + (velo_error - prev_velo_error) * vkD + (velo_integral * vkI);
	prev_velo_error = velo_error;

	// calculate turn
	turn = FF + posPID + veloPID;
	printf("FB : P%.2f, V%.2f\n", posPID, veloPID);
	return {turn, -turn};
}

bool ProfiledTurn::isSettled(kinState state) {
	double abs_error = std::abs(_targetHeading - state.position.getTheta());
	return abs_error < threshold;
}