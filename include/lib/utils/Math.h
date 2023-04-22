#pragma once
#include "lib/geometry/TrapezoidProfile.h"
#include <cmath>
#include <numbers>

namespace util {
	template<class T>
	T normalize(T val, T range) {
		return std::fmod((std::fmod(val, range) + range), range);
	}

	template<typename T>
	constexpr T lerp(const T& startValue, const T& endValue, double t) {
		return startValue + (endValue - startValue) * t;
	}

	constexpr double toRad(double deg) {
		return deg * (std::numbers::pi / 180);
	}

	constexpr double toDeg(double rad) {
		return rad * (180 / std::numbers::pi);
	}

	// all values are in degrees
	constexpr double getShortestAngle(double curHeading, double targetHeading) {
		curHeading = std::fmod(curHeading, 180) - 180.0 * std::round(curHeading / 360);
		double headingErr = targetHeading - curHeading;
		if (std::fabs(headingErr) > 180) { headingErr = headingErr > 0 ? headingErr - 360 : headingErr + 360; }

		return headingErr;
	}

	// clamps degrees to +-180
	constexpr double clampDegrees(double deg) {
		return std::fmod(deg, 180.0) - 180.0 * std::round(deg / (360.0));
	}

	template<class T>
	constexpr bool fpEquality(T a, T b) {
		return std::abs(a - b) < std::numeric_limits<T>::epsilon();
	}

	template<class T>
	constexpr bool fpLessThanEquals(T a, T b) {
		return fpEquality(a, b) || a < b;
	}

	template<class T>
	constexpr int sign(T n) {
		return (n >= 0) - (n < 0);
	}
}// namespace util