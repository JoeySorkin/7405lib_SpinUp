#pragma once

namespace ports {
	constexpr int frontLeftMotor = 3;
	constexpr int frontRightMotor = 1;
	constexpr int middleLeftMotor = 8;
	constexpr int middleRightMotor = 5;
	constexpr int backLeftMotor = 4;
	constexpr int backRightMotor = 11;

	constexpr int leftRotation = 12;
	constexpr int rightRotation = 20;
	constexpr int backRotation = 13;
}// namespace ports

namespace odometers {
	constexpr double trackWidth = 10.705;
	constexpr double leftDeadwheelDiameter = 3.25 * (1.0 / 1.375);// 1.375 is gear ratio
	constexpr double rightDeadwheelDiameter = 3.25 * (1.0 / 1.375);
	constexpr double backDeadwheelDiameter = 2.75;
	constexpr double backOffset = 1;
}// namespace odometers

namespace chassis {
	constexpr double maxVel = 1;
	constexpr double accel = 1;
	constexpr double deccel = 1;

	constexpr double maxOmega = 1;
	constexpr double maxAlphaUp = 1;
	constexpr double maxAlphaDown = 1;
}// namespace chassis