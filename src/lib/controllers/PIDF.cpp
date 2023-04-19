#include "lib/controllers/PIDF.h"
#include "lib/utils/Math.h"
#include <cmath>

PIDF::PIDF(PIDF::Constants constants) {
	setGains(constants);
	_error_integral = 0;
	_integral_cap = INFINITY;
	_integral_bound = INFINITY;
}
void PIDF::resetIntegral() {
	_error_integral = 0;
}

void PIDF::setSetpoint(double setpoint) {
	if (_f_gain_scheduler.has_value()) {
		LUT table = _f_gain_scheduler.value();
		double new_kF = table.get_val(setpoint);
		// printf("LUT: %.2f, %.2f", setpoint, new_kF);
		_kF = new_kF;
	}
	_setpoint = setpoint;
	resetIntegral();
}
double PIDF::calculate(double error, double delta_time) {
	constants_mutex.take(TIMEOUT_MAX);
	double d_error = (invert_d ? -1 : 1) * (error - _last_error) / delta_time;// try curr_rpm - last_rpm

	if (fabs(error) < _integral_bound) { _error_integral += error; }

	if (fabs(_error_integral) > _integral_cap) {
		//    _error_integral = std::signbit(_error_integral) * _integral_cap;
		_error_integral = copysign(_integral_cap, _error_integral);
	}

	//  printf("P:%.2f,I:%.2f,D:%.2f,F:%.2f,Error:%.2f\n", (_kP * error),
	//         (_kI * _error_integral), (_kD * d_error), (_kF * _setpoint),
	//         error);

	double output = (_kP * error) + (_kI * _error_integral) + (_kD * d_error) + (_kF * _setpoint);

	_last_error = error;
	constants_mutex.give();
	// pros::lcd::print(6, "%.2f integral:  %.2f", _kI, _error_integral);
	return output;
}
void PIDF::setIntegralCap(double max) {
	_integral_cap = max;
}
void PIDF::setIntegralBound(double min) {
	_integral_bound = min;
}
void PIDF::setGains(PIDF::Constants new_gains) {
	constants_mutex.take(TIMEOUT_MAX);
	_kP = new_gains.P;
	_kI = new_gains.I;
	_kD = new_gains.D;
	_kF = new_gains.F;// (Voltage / Settle RPM)
	constants_mutex.give();
}
PIDF::Constants PIDF::getGains() {
	constants_mutex.take(TIMEOUT_MAX);
	auto ret = PIDF::Constants({_kP, _kI, _kD, _kF});
	constants_mutex.give();
	return ret;
}
void PIDF::setConstants(PIDF::Constants new_consts) {
	constants_mutex.take(TIMEOUT_MAX);
	_kF = new_consts.F;
	_kP = new_consts.P;
	_kI = new_consts.I;
	_kD = new_consts.D;
	constants_mutex.give();
}
void PIDF::invertD() {
	invert_d = true;
}
