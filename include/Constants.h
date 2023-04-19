#pragma once
#include <cstdint>
#include <initializer_list>

#ifdef TEAM_R
namespace ports {
	constexpr int frontLeftMotor = 3;
	constexpr int frontRightMotor = 1;
	constexpr int middleLeftMotor = 8;
	constexpr int middleRightMotor = 5;
	constexpr int backLeftMotor = 4;
	constexpr int backRightMotor = 11;

	// numbers indicate ports, negative numbers indicate reversed motors
	constexpr std::initializer_list<std::int8_t> intake = {};

	// constexpr int leftRotation = 12;
	// constexpr int rightRotation = 20;
	constexpr int leftRotation = 0;
	constexpr int rightRotation = 0;
	constexpr int backRotation = 13;
}// namespace ports

namespace odometers {
	constexpr double trackWidth = 10.705;
	constexpr double leftDeadwheelDiameter = 3.25 * (1.0 / 1.375);// 1.375 is gear ratio
	constexpr double rightDeadwheelDiameter = 3.25 * (1.0 / 1.375);
	constexpr double backDeadwheelDiameter = 2.75;// ignored if backRotation is 0
	constexpr double backOffset = 1;              // ignored if backRotation is 0
}// namespace odometers

namespace chassis {
	constexpr double maxVel = 1;
	constexpr double accel = 1;
	constexpr double deccel = 1;

	constexpr double maxOmega = 1;
	constexpr double maxAlphaUp = 1;
	constexpr double maxAlphaDown = 1;
}// namespace chassis
#elifdef TEAM_K
namespace ports {
	constexpr int frontLeftMotor = 11;// reverse
	constexpr int frontRightMotor = 15;
	constexpr int middleLeftMotor = 3;
	constexpr int middleRightMotor = 7;// reverse
	constexpr int backLeftMotor = 13;  // reverse
	constexpr int backRightMotor = 17;

	constexpr int flywheelMotor = 5;

	// numbers indicate ports, negative numbers indicate reversed motors
	constexpr std::initializer_list<std::int8_t> intake = {-20};// determine if we reverese it or not later

	constexpr int leftRotation = 6;
	constexpr int rightRotation = 18;
	constexpr int backRotation = 0;
}// namespace ports

namespace odometers {
	// constexpr double trackWidth = 5.59265046;
	constexpr double trackWidth = 5.6030486109;
	constexpr double leftDeadwheelDiameter = 2.75;
	constexpr double rightDeadwheelDiameter = 2.75;
	constexpr double backDeadwheelDiameter = 0;// ignored if backRotation is 0
	constexpr double backOffset = 0;           // ignored if backRotation is 0
}// namespace odometers

namespace chassis {
	// maybe try like 66 in/S for vel
	constexpr double maxVel = 64.7953484803;// in/S
	// guessing these vals
	constexpr double accel = 200; // in/S^2
	constexpr double deccel = 300;// in/S^2

	constexpr double maxOmega = 1;
	constexpr double maxAlphaUp = 1;
	constexpr double maxAlphaDown = 1;
}// namespace chassis
#else
#error Constants for either TEAM_K or TEAM_R have not been selected.
#endif