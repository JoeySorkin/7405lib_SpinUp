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

	constexpr int scataRotation = 8;
	constexpr int scataMotor = 19;

	constexpr int leftRotation = 0;
	constexpr int rightRotation = 0;
	constexpr int backRotation = 7;

	constexpr int rightExpansion = 0;
	constexpr int leftExpansion = 0;

constexpr std::initializer_list<std::int8_t> intake = {6};
}// namespace ports


namespace odometers {
	constexpr double trackWidth = 10.705;
	constexpr double leftDeadwheelDiameter = 3.25 * (1.0 / 1.375);// 1.375 is gear ratio
	constexpr double rightDeadwheelDiameter = 3.25 * (1.0 / 1.375);
	constexpr double backDeadwheelDiameter = 2.75;// ignored if backRotation is 0
	constexpr double backOffset = 1;              // ignored if backRotation is 0
}// namespace odometers

namespace chassis {
	constexpr double maxVel = 71.795864;
	constexpr double accel = 87.8518519;
	constexpr double deccel = 87.8518519;

	constexpr double maxOmega = 1;
	constexpr double maxAlphaUp = 1;
	constexpr double maxAlphaDown = 1;
}// namespace chassis

namespace scata {
	constexpr int lowPowerThreshold = -300;
	constexpr int stopThreshold = -160;

	constexpr int lowPower = 7500;
	constexpr int highPower = 12000;

	constexpr std::initializer_list<std::int8_t> leftBoost = {5, 'B'}; //need to check which one is left and right
	constexpr std::initializer_list<std::int8_t> rightBoost = {5, 'H'};
	constexpr std::initializer_list<std::int8_t> toggleBoost = {5, 'A'};
}