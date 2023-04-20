#pragma once
#include <cstdint>
#include <initializer_list>


namespace ports {
	constexpr int frontLeftMotor = 1;
	constexpr int frontRightMotor = 2;
	constexpr int middleLeftMotor = 9;
	constexpr int middleRightMotor = 3;
	constexpr int backLeftMotor = 10;
	constexpr int backRightMotor = 16;

	// constexpr int frontRightMotor = 1;
	// constexpr int frontLeftMotor = 2;
	// constexpr int middleRightMotor = 9;
	// constexpr int middleLeftMotor = 3;
	// constexpr int backRightMotor = 10;
	// constexpr int backLeftMotor = 16;
	constexpr std::initializer_list<std::int8_t> intake = {};

	constexpr int leftRotation = 0;
	constexpr int rightRotation = 0;
	constexpr int backRotation = 7;
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
