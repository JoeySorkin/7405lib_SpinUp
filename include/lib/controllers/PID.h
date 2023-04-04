#pragma once
#include <limits>

// i love register spilling
class PID {
private:
	double kP;
	double kI;
	double kD;
	double iBound;

	double prevError;
	double errorSum;

	// when error sign, we set error sum to 0
	bool errorZeroFlip;

	bool resetFlag;

public:
	PID();
	explicit PID(double P, double I, double D, bool errorZeroFlip = false,
	             double iBound = std::numeric_limits<double>::infinity());

	[[nodiscard]] double operator()(double error);

	void setP(double kP);
	void setI(double kI);
	void setD(double kD);

	// probably delete - because PID objects are most likely a one time usage here
	void reset();
};