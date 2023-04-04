#pragma once

class TrapezoidProfile {
private:
	struct State {
		double pos, vel, acc;
	};

	double dist, acc_max, dec_max, v_max;
	double t_acc, t_coast, t_dec;
	double d_acc, d_coast, d_dec;

	int direction;

	double acceleration(double t) const;
	double velocity(double t) const;
	double position(double t) const;

public:
	TrapezoidProfile(double distance, double max_acceleration, double max_deceleration, double max_velocity);

	State getState(double t) const;
};