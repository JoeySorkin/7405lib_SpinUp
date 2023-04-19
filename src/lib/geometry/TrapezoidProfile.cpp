#include "lib/geometry/TrapezoidProfile.h"
#include <cmath>

TrapezoidProfile::TrapezoidProfile(double distance, double max_acceleration, double max_deceleration,
                                   double max_velocity) {
	dist = distance;
	acc_max = max_acceleration;
	dec_max = max_deceleration;
	v_max = fmin(fmin(sqrt(acc_max * dist), sqrt(dec_max * dist)), max_velocity);

	// generate variables

	// calculate ramp up and ramp down times
	t_acc = v_max / acc_max;
	t_dec = v_max / dec_max;

	// calculate the distance traveled along those ramp up times (∆x = v_0 * t +
	// 0.5 * a * t^2).. v_0 is 0 for both NOTE - dec_max is a positive
	// quantity... idk why i did that, but thats how it works. Thats why v_0 is
	// 0 for both d_acc and d_dec calculation even though technically d_dec
	// starts at maxvel and goes to 0.
	d_acc = 0.5 * acc_max * pow(t_acc, 2.0);
	d_dec = 0.5 * dec_max * pow(t_dec, 2.0);
	d_coast = dist - d_acc - d_dec;

	t_coast = d_coast / v_max;

	direction = dist < 0 ? -1 : 1;
}

// can get rid of these three functions and combine them into one func
// and just have 4 branches
double TrapezoidProfile::acceleration(double t) const {
	if (t <= t_acc) {
		return acc_max;
	} else if (t >= (t_acc + t_coast) && t <= (t_acc + t_coast + t_dec)) {
		return -dec_max;
	} else {
		return 0.0;
	}
}
double TrapezoidProfile::velocity(double t) const {
	if (t <= t_acc) {
		return acc_max * t;
	} else if (t <= (t_acc + t_coast)) {
		return v_max;
	} else if (t <= (t_acc + t_coast + t_dec)) {
		return -1 * dec_max * (t - (t_acc + t_coast)) + v_max;
	} else {
		return 0.0;
	}
}
double TrapezoidProfile::position(double t) const {
	if (t <= t_acc) {
		return 0.5 * acc_max * pow(t, 2.0);
	} else if (t <= (t_acc + t_coast)) {
		return v_max * t - position(t_acc);
	} else if (t <= (t_acc + t_coast + t_dec)) {
		return -0.5 * dec_max * pow(t - t_acc - t_coast - t_dec, 2.0) + position(t_acc + t_coast) + d_dec;
	} else {
		return dist;
	}
}

TrapezoidProfile::State TrapezoidProfile::getState(double t) const {
	// State state{0};
	// if (t <= t_acc) {
	// 	state.acc = acc_max;
	// 	state.vel = acc_max * t;
	// 	state.pos = (0.5 * acc_max * t) * t;
	// } else if (t <= (t_acc + t_coast)) {
	// 	state.acc = 0;
	// 	state.vel = v_max;
	// 	state.pos = (t_acc * acc_max * 0.5) * t_acc + v_max * (t - t_acc);
	// } else if (t <= (t_acc + t_coast + t_dec)) {
	// 	double timeLeft = (t_acc + t_coast + t_dec) - t;
	// 	state.acc = -dec_max;
	// 	state.vel = ;
	// }
	return {position(t) * direction, velocity(t) * direction, acceleration(t) * direction};
}

double TrapezoidProfile::getTargetDist() const {
	return dist;
}

double TrapezoidProfile::getTotalTime() const {
	return t_acc + t_coast + t_dec;
}