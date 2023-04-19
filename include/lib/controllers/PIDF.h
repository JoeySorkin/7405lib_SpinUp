#pragma once
#include "lib/utils/LUT.h"
#include "pros/rtos.hpp"
#include <atomic>
#include <optional>

class PIDF {
public:
	struct Constants {
		double P, I, D, F;
	};

	PIDF() = default;
	PIDF(Constants constants);
	void setSetpoint(double setpoint);
	double calculate(double error, double delta_time = 20);
	void setGains(Constants new_gains);
	void invertD();
	void printPlotable();
	Constants getGains();
	/**
	 * @param max the maximum value the absolute val of the integral can
	 * accumulate to
	 */
	void setIntegralCap(double max);
	/**
	 * @param min the minimum value of error for integral to begin accumulating
	 */
	void setIntegralBound(double min);

	void setConstants(Constants new_consts);

private:
	void resetIntegral();
	std::optional<LUT> _f_gain_scheduler;
	pros::Mutex constants_mutex;
	double _kP, _kI, _kD, _kF;
	double _setpoint;
	double _error_integral, _integral_cap, _integral_bound;
	double _last_error;
	bool invert_d = false;
};